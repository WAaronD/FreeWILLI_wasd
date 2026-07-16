#include "processing_stages.h"

#include "../algorithms/localization/hydrophone_position_processing.h"

DataAcquisitionStage::DataAcquisitionStage(SharedDataManager& manager) : mSharedDataManager(manager) {}

bool DataAcquisitionStage::process(std::shared_ptr<ProcessingContext> context)
{
    mSharedDataManager.waitForData(context->dataBytes, context->firmware->numPacketsToDetect());

    context->dataTimes = context->firmware->generateTimestamp(context->dataBytes);

    // Validate data integrity
    context->firmware->throwIfDataErrors(context->dataBytes, mPreviousTimeSet, mPreviousTime, context->dataTimes);

    // Update previous time tracking
    if (!context->dataTimes.empty())
    {
        mPreviousTime = context->dataTimes.back();
        mPreviousTimeSet = true;
    }

    context->firmware->insertDataIntoChannelMatrix(context->channelData, context->dataBytes);
    // std::cout << "[DataAcquisition] channelData: " << context->channelData.rows() // for debug
    //       << "x" << context->channelData.cols() << std::endl;

    return true;
}

std::string DataAcquisitionStage::getName() const { return "Data Acquisition"; }

void DataAcquisitionStage::initialize(std::shared_ptr<ProcessingContext> context) {}

TimeDomainDetectionStage::TimeDomainDetectionStage(std::unique_ptr<ITimeDomainDetector> detector)
    : mFunction(std::move(detector))
{
}

bool TimeDomainDetectionStage::process(std::shared_ptr<ProcessingContext> context)
{
    bool detected = mFunction->detect(context->channelData.row(0));

    if (detected)
    {
        context->currentResult.peakAmplitude = mFunction->getLastDetection();
        context->currentResult.isValid = true;

        // New! Populate oc if this is a RuCCUSDetector
        if (auto* ruccus = dynamic_cast<RuCCUSDetector*>(mFunction.get()))
        {
            context->currentResult.oscillationCount = ruccus->getLastExcursionCount();
        }

        std::cout << "Peak amplitude = " << context->currentResult.peakAmplitude << std::endl;
    }

    return detected;
}

std::string TimeDomainDetectionStage::getName() const { return "Time Domain Detection"; }

TimeDomainFilteringStage::TimeDomainFilteringStage(std::unique_ptr<ITimeDomainFilter> filter)
    : mFunction(std::move(filter))
{
}

bool TimeDomainFilteringStage::process(std::shared_ptr<ProcessingContext> context)
{
    (void)context;  // silence unused-parameter

    if (!mFunction)
    {
        return false;
    }

    // Apply frequency domain filter
    mFunction->apply();

    return true;
}

std::string TimeDomainFilteringStage::getName() const { return "Time Domain Filtering"; }

FrequencyDomainFilteringStage::FrequencyDomainFilteringStage(std::unique_ptr<IFrequencyDomainTransform> filter)
    : mFunction(std::move(filter))
{
}

bool FrequencyDomainFilteringStage::process(std::shared_ptr<ProcessingContext> context)
{
    if (!mFunction)
    {
        return false;
    }

    // Apply frequency domain filter
    mFunction->apply();
    std::cout << "Frequency spectrum:\n" << mFunction->getFrequencyDomainData() << std::endl; // for debug

    auto& fd = context->frequencyDomainData; // for debug
    std::cout << "[FreqDomainFilter apply] bin0=" << fd(0,0) << " bin100=" << fd(100,0) << std::endl; // for debug

    // Store filtered and unfiltered frequency domain data
    context->frequencyDomainData = mFunction->getFrequencyDomainData();
    context->beforeFilterData = mFunction->mBeforeFilter;
    return true;
}

std::string FrequencyDomainFilteringStage::getName() const { return "Frequency Domain Filtering"; }

FrequencyDomainDetectionStage::FrequencyDomainDetectionStage(std::unique_ptr<IFrequencyDomainDetector> detector)
    : mFunction(std::move(detector))
{
}

bool FrequencyDomainDetectionStage::process(std::shared_ptr<ProcessingContext> context)
{
    if (!mFunction || context->frequencyDomainData.cols() == 0)
    {
        return true;
    }

    bool detected = mFunction->detect(context->frequencyDomainData.col(0));

    // New! Populate log10sr if this is a RuCCUSFDetector
    if (auto* ruccusF = dynamic_cast<RuCCUSFDetector*>(mFunction.get()))
    {
        context->currentResult.log10SpectrumRatio = ruccusF->getLastSpectrumRatio();
    }

    return detected;
}

std::string FrequencyDomainDetectionStage::getName() const { return "FrequencyDomainDetection"; }

ONNXClassificationStage::ONNXClassificationStage(std::unique_ptr<ONNXModel> model, size_t spectraSize)
    : mFunction(std::move(model)), mSpectraSize(spectraSize)
{
}

bool ONNXClassificationStage::process(std::shared_ptr<ProcessingContext> context)
{
    if (!mFunction || context->beforeFilterData.rows() == 0)
    {
        return true;  // Skip if no model or no data
    }

    try
    {
        // Extract magnitude spectra for inference
        Eigen::VectorXf spectraToInference = context->beforeFilterData.array().abs();

        // Take the last mSpectraSize samples
        if (spectraToInference.size() < static_cast<int>(mSpectraSize))
        {
            std::cerr << "Warning: Insufficient spectra data for classification\n";
            return true;  // Continue processing
        }

        Eigen::VectorXf spectraForInference = spectraToInference.tail(mSpectraSize);

        // Convert to vector for ONNX model
        std::vector<float> spectraVector(
            spectraForInference.data(), spectraForInference.data() + spectraForInference.size());

        // Run inference
        std::vector<float> output = mFunction->runInference(spectraVector);

        // Check classification result (assuming binary classification: [noise, signal])
        if (output.size() >= 2 && output[1] < output[0])
        {
            std::cout << "Noise detected by classifier, skipping detection\n";
            return false;  // Skip further processing for noise
        }

        return true;  // Continue processing for signal
    }
    catch (const std::exception& e)
    {
        std::cerr << "Classification error: " << e.what() << ", continuing without classification\n";
        return true;  // Continue processing on classification error
    }
}

std::string ONNXClassificationStage::getName() const { return "Classification"; }

DirectionEstimationStage::DirectionEstimationStage(
    std::unique_ptr<GCC_PHAT> gccPhat, const Eigen::MatrixXf& cachedLS, int rank)
    : mFunction(std::move(gccPhat)), mCachedLeastSquares(cachedLS), mHydrophoneMatrixRank(rank)
{
}

bool DirectionEstimationStage::process(std::shared_ptr<ProcessingContext> context)
{
    if (context->frequencyDomainData.cols() == 0)
    {
        return false;  // No frequency domain data available
    }

    try
    {
        // Measure GCC-PHAT processing time
        auto beforeGCC = std::chrono::steady_clock::now();

        // Compute TDOAs and cross-correlation amplitudes
        auto [tdoaVector, xCorrAmps] = mFunction->process(context->frequencyDomainData);

        auto afterGCC = std::chrono::steady_clock::now();
        std::chrono::duration<double> duration = afterGCC - beforeGCC;
        std::cout << "GCC-PHAT processing time: " << duration.count() << " seconds\n";

        // Store intermediate results
        // context->tdoaVector = tdoaVector;
        // context->crossCorrelationAmps = xCorrAmps;

        // Compute direction of arrival from TDOAs
        Eigen::VectorXf directionOfArrival = computeDoaFromTdoa(mCachedLeastSquares, tdoaVector, mHydrophoneMatrixRank);

        // context->directionOfArrival = directionOfArrival;

        // Convert to azimuth and elevation for display
        Eigen::VectorXf azimuthAndElevation = convertDoaToElAz(directionOfArrival);
        std::cout << "Azimuth/Elevation: " << azimuthAndElevation.transpose() << std::endl;

        // Populate the detection result
        context->currentResult.directionOfArrival = directionOfArrival.head<3>();
        context->currentResult.tdoaVector = tdoaVector;
        context->currentResult.crossCorrelationAmps = xCorrAmps;
        context->currentResult.isValid = true;

        return true;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Direction estimation error: " << e.what() << std::endl;
        return false;
    }
}

std::string DirectionEstimationStage::getName() const { return "DirectionEstimation"; }

TrackingStage::TrackingStage(std::unique_ptr<Tracker> tracker) : mFunction(std::move(tracker)) {}

bool TrackingStage::process(std::shared_ptr<ProcessingContext> context)
{
    if (!mFunction)
    {
        return true;  // Skip if no tracker configured
    }

    try
    {
        // Update tracker buffer with new direction
        mFunction->updateTrackerBuffer(context->currentResult.directionOfArrival);

        // Update Kalman filters if tracker is initialized
        if (mFunction->mIsTrackerInitialized && !context->dataTimes.empty())
        {
            int trackingLabel = mFunction->updateKalmanFiltersContinuous(
                context->currentResult.directionOfArrival, context->dataTimes[0]);

            // Store tracking label in result
            context->currentResult.trackingLabel = trackingLabel;
        }

        return true;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Tracking error: " << e.what() << ", continuing without tracking\n";
        return true;  // Continue processing even if tracking fails
    }
}

std::string TrackingStage::getName() const { return "Tracking"; }

void TrackingStage::initialize(std::shared_ptr<ProcessingContext> context)
{
    if (mFunction && !context->dataTimes.empty())
    {
        // Initialize tracker output file with first timestamp
        mFunction->initializeOutputFile(context->dataTimes[0]);
    }
}

bool TrackingStage::requiresPeriodicTick() const { return true; }

void TrackingStage::tick()
{
    if (mFunction)
    {
        mFunction->scheduleCluster();
    }
}