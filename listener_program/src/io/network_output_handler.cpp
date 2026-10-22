#include "../processing_context_struct.h"
#include "output_handlers.h"

namespace {
    std::string optionalToString(const std::optional<float>& val)
    {
        return val.has_value() ? std::to_string(*val) : "NaN";
    }
    std::string optionalToString(const std::optional<std::string>& val)
    {
        return val.has_value() ? *val : "";
    }
    std::string matchesToString(const std::optional<std::vector<bool>>& matches)
    {
        if (!matches.has_value()) return "";
        std::string s;
        s.reserve(matches->size());
        for (bool b : *matches) s += (b ? '1' : '0');
        return s;
    }

    std::vector<std::string> generateChannelComboLabels(const std::string& labelPrefix, int numChannels)
    {
        std::vector<std::string> labels;
        for (int signalChannel = 1; signalChannel < numChannels; ++signalChannel)
        {
            for (int referenceChannel = signalChannel + 1; referenceChannel <= numChannels; ++referenceChannel)
            {
                labels.push_back(labelPrefix + std::to_string(signalChannel * 10 + referenceChannel));
            }
        }
        return labels;
    }
}

NetworkOutputHandler::NetworkOutputHandler(const std::string& ip, int port) : mIp(ip), mPort(port), mSockfd(-1)
{
    // 1) create socket
    mSockfd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (mSockfd < 0)
    {
        throw std::runtime_error("NetworkOutputHandler: socket() failed");
    }

    // 2) fill out destination address struct
    std::memset(&mDest, 0, sizeof(mDest));
    mDest.sin_family = AF_INET;
    mDest.sin_port = htons(mPort);
    if (::inet_pton(AF_INET, mIp.c_str(), &mDest.sin_addr) != 1)
    {
        ::close(mSockfd);
        throw std::invalid_argument("NetworkOutputHandler: invalid IP address");
    }
}

NetworkOutputHandler::~NetworkOutputHandler()
{
    if (mSockfd >= 0)
    {
        ::close(mSockfd);
    }
}

void NetworkOutputHandler::initialize(const TimePoint& timestamp, int numChannels)
{
    mInitTimestamp = timestamp;
    mInitNumChannels = numChannels;

    std::vector<std::string> columnNames = {
        "PeakTime", "Amplitude", "DOA_x", "DOA_y", "DOA_z", "OC", "Log10SR",
        "SignalDuration", "PeakFreq", "CenterFreq", "ClassLabel", "ClassProb",
        "DurationMatches", "PeakMatches", "CenterMatches", "FinalAssignment"};

    auto tdoaLabels = generateChannelComboLabels("TDOA", numChannels);
    auto xcorrLabels = generateChannelComboLabels("XCorr", numChannels);
    columnNames.insert(columnNames.end(), tdoaLabels.begin(), tdoaLabels.end());
    columnNames.insert(columnNames.end(), xcorrLabels.begin(), xcorrLabels.end());

    mHeaderRow.clear();
    for (size_t i = 0; i < columnNames.size(); ++i)
    {
        mHeaderRow += columnNames[i];
        if (i < columnNames.size() - 1) mHeaderRow += ',';
    }
}

void NetworkOutputHandler::handleOutput(const ProcessingContext& result)
{
    if (!result.currentResult.isValid)
    {
        return;
    }

    const DetectionResult& r = result.currentResult;
    std::vector<std::string> f;

    auto timeSinceEpoch =
        std::chrono::duration_cast<std::chrono::microseconds>(result.dataTimes[0].time_since_epoch());
    f.push_back(std::to_string(timeSinceEpoch.count()));

    f.push_back(std::to_string(r.peakAmplitude));
    f.push_back(std::to_string(r.directionOfArrival.x()));
    f.push_back(std::to_string(r.directionOfArrival.y()));
    f.push_back(std::to_string(r.directionOfArrival.z()));

    f.push_back(optionalToString(r.oscillationCount));
    f.push_back(optionalToString(r.log10SpectrumRatio));
    f.push_back(optionalToString(r.signalDuration));
    f.push_back(optionalToString(r.peakFrequency));
    f.push_back(optionalToString(r.centerFrequency));
    f.push_back(optionalToString(r.classLabel));
    f.push_back(optionalToString(r.classProbability));
    f.push_back(matchesToString(r.signalDurationMatches));
    f.push_back(matchesToString(r.peakFreqMatches));
    f.push_back(matchesToString(r.centerFreqMatches));
    f.push_back(matchesToString(r.agreedMatches));

    for (int j = 0; j < r.tdoaVector.size(); ++j) f.push_back(std::to_string(r.tdoaVector[j]));
    for (int j = 0; j < r.crossCorrelationAmps.size(); ++j) f.push_back(std::to_string(r.crossCorrelationAmps[j]));

    // Packet = header row + newline + data row + newline (same as first two lines of the file)
    std::string packet = mHeaderRow + "\n";
    for (size_t k = 0; k < f.size(); ++k)
    {
        packet += f[k];
        if (k < f.size() - 1) packet += ',';
    }
    packet += "\n";

    ssize_t sent = ::sendto(
        mSockfd, packet.data(), packet.size(), 0, reinterpret_cast<struct sockaddr*>(&mDest), sizeof(mDest));
    if (sent < 0)
    {
        throw std::runtime_error("NetworkOutputHandler: sendto() failed");
    }
}

void NetworkOutputHandler::finalize()
{
    // no-op
}