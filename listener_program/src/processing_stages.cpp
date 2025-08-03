#include "processing_stages.h"

#include <chrono>
#include <iostream>

#include "algorithms/hydrophone_position_processing.h"

// ============================================================================
// DATA ACQUISITION STAGE
// ============================================================================

DataAcquisitionStage::DataAcquisitionStage(SharedDataManager& manager, std::shared_ptr<const IFirmware> firmware)
    : mSharedDataManager(manager), mFirmware(firmware)
{
}

bool DataAcquisitionStage::process(std::shared_ptr<ProcessingContext> context)
{
    // Wait for data from shared manager
    mSharedDataManager.waitForData(context->dataBytes, mFirmware->numPacketsToDetect());

    // Generate timestamps for the data
    context->dataTimes = mFirmware->generateTimestamp(context->dataBytes);

    // Validate data integrity
    mFirmware->throwIfDataErrors(context->dataBytes, mPreviousTimeSet, mPreviousTime, context->dataTimes);

    // Update previous time tracking
    if (!context->dataTimes.empty())
    {
        mPreviousTime = context->dataTimes.back();
        mPreviousTimeSet = true;
    }

    // Insert data into channel matrix
    mFirmware->insertDataIntoChannelMatrix(context->channelData, context->dataBytes);
    return context->pipelineInitialized;
}

std::string DataAcquisitionStage::getName() const { return "DataAcquisition"; }

void DataAcquisitionStage::initialize(std::shared_ptr<ProcessingContext> context)
{
    /*
    // Ensure channel data matrix is properly sized
    if (context.channelData.rows() != mFirmware->numChannels() ||
        context.channelData.cols() != mFirmware->channelSize())
    {
        const_cast<ProcessingContext&>(context).channelData =
            Eigen::MatrixXf::Zero(mFirmware->numChannels(), mFirmware->channelSize());
    }

    // Resize data containers
    const_cast<ProcessingContext&>(context).dataBytes.resize(mFirmware->numPacketsToDetect());
    */
}

// ============================================================================
// TIME DOMAIN DETECTION STAGE
// ============================================================================

TimeDomainDetectionStage::TimeDomainDetectionStage(std::unique_ptr<ITimeDomainDetector> detector)
    : mDetector(std::move(detector))
{
}

bool TimeDomainDetectionStage::process(std::shared_ptr<ProcessingContext> context)
{
    if (!mDetector)
    {
        return true;  // Skip if no detector configured
    }

    // Perform time domain detection on first channel
    // std::cout << context.channelData.row(0) << std::endl;
    bool detected = mDetector->detect(context->channelData.row(0));

    if (detected)
    {
        // Store detection amplitude and timestamp
        context->timeDomainDetectionValue = mDetector->getLastDetection();
        context->currentResult.peakAmplitude = mDetector->getLastDetection();
        if (!context->dataTimes.empty())
        {
            context->currentResult.timestamp = context->dataTimes[0];
        }
    }

    return detected;
}

std::string TimeDomainDetectionStage::getName() const { return "TimeDomainDetection"; }

// ============================================================================
// FILTERING STAGE
// ============================================================================

FilteringStage::FilteringStage(std::unique_ptr<IFrequencyDomainStrategy> filter) : mFilter(std::move(filter)) {}

bool FilteringStage::process(std::shared_ptr<ProcessingContext> context)
{
    if (!mFilter)
    {
        return false;
    }

    // Apply frequency domain filter
    mFilter->apply();

    // Store filtered and unfiltered frequency domain data
    context->frequencyDomainData = mFilter->getFrequencyDomainData();
    context->beforeFilterData = mFilter->mBeforeFilter;
    return true;
}

std::string FilteringStage::getName() const { return "Filtering"; }

// ============================================================================
// FREQUENCY DOMAIN DETECTION STAGE
// ============================================================================

FrequencyDomainDetectionStage::FrequencyDomainDetectionStage(std::unique_ptr<IFrequencyDomainDetector> detector)
    : mDetector(std::move(detector))
{
}

bool FrequencyDomainDetectionStage::process(std::shared_ptr<ProcessingContext> context)
{
    if (!mDetector || context->frequencyDomainData.cols() == 0)
    {
        return true;  // Skip if no detector or no frequency data
    }

    // Perform frequency domain detection on first channel
    return mDetector->detect(context->frequencyDomainData.col(0));
}

std::string FrequencyDomainDetectionStage::getName() const { return "FrequencyDomainDetection"; }

// ============================================================================
// CLASSIFICATION STAGE
// ============================================================================

ClassificationStage::ClassificationStage(std::unique_ptr<ONNXModel> model, size_t spectraSize)
    : mModel(std::move(model)), mSpectraSize(spectraSize)
{
}

bool ClassificationStage::process(std::shared_ptr<ProcessingContext> context)
{
    if (!mModel || context->beforeFilterData.rows() == 0)
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
        std::vector<float> output = mModel->runInference(spectraVector);

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

std::string ClassificationStage::getName() const { return "Classification"; }

// ============================================================================
// DIRECTION ESTIMATION STAGE
// ============================================================================

DirectionEstimationStage::DirectionEstimationStage(
    std::unique_ptr<GCC_PHAT> gccPhat, const Eigen::MatrixXf& cachedLS, int rank)
    : mComputeTDOAs(std::move(gccPhat)), mCachedLeastSquares(cachedLS), mHydrophoneMatrixRank(rank)
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
        auto [tdoaVector, xCorrAmps] = mComputeTDOAs->process(context->frequencyDomainData);

        auto afterGCC = std::chrono::steady_clock::now();
        std::chrono::duration<double> duration = afterGCC - beforeGCC;
        std::cout << "GCC-PHAT processing time: " << duration.count() << " seconds\n";

        // Store intermediate results
        context->tdoaVector = tdoaVector;
        context->crossCorrelationAmps = xCorrAmps;

        // Compute direction of arrival from TDOAs
        Eigen::VectorXf directionOfArrival = computeDoaFromTdoa(mCachedLeastSquares, tdoaVector, mHydrophoneMatrixRank);

        context->directionOfArrival = directionOfArrival;

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

// ============================================================================
// TRACKING STAGE
// ============================================================================

TrackingStage::TrackingStage(std::unique_ptr<Tracker> tracker) : mTracker(std::move(tracker)) {}

bool TrackingStage::process(std::shared_ptr<ProcessingContext> context)
{
    if (!mTracker)
    {
        return true;  // Skip if no tracker configured
    }

    try
    {
        // Update tracker buffer with new direction
        mTracker->updateTrackerBuffer(context->directionOfArrival);

        // Update Kalman filters if tracker is initialized
        if (mTracker->mIsTrackerInitialized && !context->dataTimes.empty())
        {
            int trackingLabel =
                mTracker->updateKalmanFiltersContinuous(context->directionOfArrival, context->dataTimes[0]);

            // Store tracking label in result
            context->currentResult.trackingLabel = trackingLabel;

            // Optional: Save spectra for training
            // if (context.beforeFilterData.rows() > 0) {
            //     mOutputManager->saveSpectraForTraining(
            //         "training_data.csv", trackingLabel, context.beforeFilterData
            //     );
            // }>>
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
    std::cout << "mTracker: " << mTracker << std::endl;
    std::cout << "!context.dataTimes.empty(): " << !context->dataTimes.empty() << std::endl;
    if (mTracker && !context->dataTimes.empty())
    {
        // Initialize tracker output file with first timestamp
        std::cout << "before tracker init: " << " " << std::endl;
        mTracker->initializeOutputFile(context->dataTimes[0]);
        std::cout << "after tracker init: " << std::endl;
    }
}

bool TrackingStage::requiresPeriodicTick() const { return true; }

void TrackingStage::tick()
{
    // Schedule cluster processing
    mTracker->scheduleCluster();
}