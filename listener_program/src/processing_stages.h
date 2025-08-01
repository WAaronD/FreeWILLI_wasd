#pragma once

#include "ML/onnx_model.h"
#include "algorithms/doa_utils.h"
#include "algorithms/fir_filter_factory.h"
#include "algorithms/frequency_domain_detectors_factory.h"
#include "algorithms/gcc_phat.h"
#include "algorithms/time_domain_detectors_factory.h"
#include "firmware/firmware_interface.h"
#include "interfaces.h"
#include "shared_data_manager.h"
#include "tracker/tracker.h"

// ============================================================================
// PROCESSING STAGE IMPLEMENTATIONS
// ============================================================================

class DataAcquisitionStage : public IProcessingStage
{
   private:
    SharedDataManager& mSharedDataManager;
    std::shared_ptr<const IFirmware> mFirmware;
    bool mPreviousTimeSet = false;
    TimePoint mPreviousTime = TimePoint::min();

   public:
    DataAcquisitionStage(SharedDataManager& manager, std::shared_ptr<const IFirmware> firmware);
    bool process(std::shared_ptr<ProcessingContext> context) override;
    std::string getName() const override;
    void initialize(std::shared_ptr<ProcessingContext> context) override;
};

class TimeDomainDetectionStage : public IProcessingStage
{
   private:
    std::unique_ptr<ITimeDomainDetector> mDetector;

   public:
    explicit TimeDomainDetectionStage(std::unique_ptr<ITimeDomainDetector> detector);
    bool process(std::shared_ptr<ProcessingContext> context) override;
    std::string getName() const override;
};

class FilteringStage : public IProcessingStage
{
   private:
    std::unique_ptr<IFrequencyDomainStrategy> mFilter;

   public:
    explicit FilteringStage(std::unique_ptr<IFrequencyDomainStrategy> filter);
    bool process(std::shared_ptr<ProcessingContext> context) override;
    std::string getName() const override;
};

class FrequencyDomainDetectionStage : public IProcessingStage
{
   private:
    std::unique_ptr<IFrequencyDomainDetector> mDetector;

   public:
    explicit FrequencyDomainDetectionStage(std::unique_ptr<IFrequencyDomainDetector> detector);
    bool process(std::shared_ptr<ProcessingContext> context) override;
    std::string getName() const override;
};

class ClassificationStage : public IProcessingStage
{
   private:
    std::unique_ptr<ONNXModel> mModel;
    size_t mSpectraSize;

   public:
    ClassificationStage(std::unique_ptr<ONNXModel> model, size_t spectraSize = 500);
    bool process(std::shared_ptr<ProcessingContext> context) override;
    std::string getName() const override;
};

class DirectionEstimationStage : public IProcessingStage
{
   private:
    std::unique_ptr<GCC_PHAT> mComputeTDOAs;
    Eigen::MatrixXf mCachedLeastSquares;
    int mHydrophoneMatrixRank;

   public:
    DirectionEstimationStage(std::unique_ptr<GCC_PHAT> gccPhat, const Eigen::MatrixXf& cachedLS, int rank);
    bool process(std::shared_ptr<ProcessingContext> context) override;
    std::string getName() const override;
};

class TrackingStage : public IProcessingStage
{
   private:
    std::unique_ptr<Tracker> mTracker;

   public:
    explicit TrackingStage(std::unique_ptr<Tracker> tracker);
    bool process(std::shared_ptr<ProcessingContext> context) override;
    std::string getName() const override;
    void initialize(std::shared_ptr<ProcessingContext> context) override;
};