#pragma once

#include "../ML/onnx_model.h"
#include "../algorithms/detectors/frequency_domain_detectors_factory.h"
#include "../algorithms/detectors/time_domain_detectors_factory.h"
#include "../algorithms/frequency_domain_transformation_factory.h"
#include "../algorithms/localization/doa_utils.h"
#include "../algorithms/localization/gcc_phat.h"
#include "../algorithms/time_domain_filters.h"
#include "../firmware/firmware_interface.h"
#include "../shared_data_manager.h"
#include "../tracker/tracker.h"
#include "core_interfaces.h"

class DataAcquisitionStage : public IProcessingStage
{
   public:
    DataAcquisitionStage(SharedDataManager& manager, std::shared_ptr<const IFirmware> firmware);
    bool process(std::shared_ptr<ProcessingContext> context) override;
    std::string getName() const override;
    void initialize(std::shared_ptr<ProcessingContext> context) override;

   private:
    SharedDataManager& mSharedDataManager;
    std::shared_ptr<const IFirmware> mFirmware;
    bool mPreviousTimeSet = false;
    TimePoint mPreviousTime = TimePoint::min();
};

class TimeDomainDetectionStage : public IProcessingStage
{
   public:
    explicit TimeDomainDetectionStage(std::unique_ptr<ITimeDomainDetector> detector);
    bool process(std::shared_ptr<ProcessingContext> context) override;
    std::string getName() const override;

   private:
    std::unique_ptr<ITimeDomainDetector> mDetector;
};

class FrequencyDomainFilteringStage : public IProcessingStage
{
   public:
    explicit FrequencyDomainFilteringStage(std::unique_ptr<IFrequencyDomainTransform> filter);
    bool process(std::shared_ptr<ProcessingContext> context) override;
    std::string getName() const override;

   private:
    std::unique_ptr<IFrequencyDomainTransform> mFilter;
};

class TimeDomainFilteringStage : public IProcessingStage
{
   public:
    explicit TimeDomainFilteringStage(std::unique_ptr<ITimeDomainFilter> filter);
    bool process(std::shared_ptr<ProcessingContext> context) override;
    std::string getName() const override;

   private:
    std::unique_ptr<ITimeDomainFilter> mFilter;
};

class FrequencyDomainDetectionStage : public IProcessingStage
{
   public:
    explicit FrequencyDomainDetectionStage(std::unique_ptr<IFrequencyDomainDetector> detector);
    bool process(std::shared_ptr<ProcessingContext> context) override;
    std::string getName() const override;

   private:
    std::unique_ptr<IFrequencyDomainDetector> mDetector;
};

class ONNXClassificationStage : public IProcessingStage
{
   public:
    ONNXClassificationStage(std::unique_ptr<ONNXModel> model, size_t spectraSize = 500);
    bool process(std::shared_ptr<ProcessingContext> context) override;
    std::string getName() const override;

   private:
    std::unique_ptr<ONNXModel> mModel;
    size_t mSpectraSize;
};

class DirectionEstimationStage : public IProcessingStage
{
   public:
    DirectionEstimationStage(std::unique_ptr<GCC_PHAT> gccPhat, const Eigen::MatrixXf& cachedLS, int rank);
    bool process(std::shared_ptr<ProcessingContext> context) override;
    std::string getName() const override;

   private:
    std::unique_ptr<GCC_PHAT> mComputeTDOAs;
    Eigen::MatrixXf mCachedLeastSquares;
    int mHydrophoneMatrixRank;
};

class TrackingStage : public IProcessingStage
{
   public:
    explicit TrackingStage(std::unique_ptr<Tracker> tracker);
    bool process(std::shared_ptr<ProcessingContext> context) override;
    std::string getName() const override;
    void initialize(std::shared_ptr<ProcessingContext> context) override;
    bool requiresPeriodicTick() const override;
    void tick() override;

   private:
    std::unique_ptr<Tracker> mTracker;
};