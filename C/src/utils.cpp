#include "utils.h"
#include "process_data.h"
#include "socket_manager.h"
#include "custom_types.h"

using TimePoint = std::chrono::system_clock::time_point;

void PrintMode() 
{
#ifdef DEBUG
    std::cout << "Running Debug Mode" << std::endl;
#else
    std::cout << "Running Release Mode" << std::endl;
#endif
}

void InitializeSession(SocketManager& sess, ExperimentRuntime& expRuntime, char* argv[]) 
{
    /**
     * @brief Initializes the Session and Experiment structures with command-line arguments.
     *
     * @param sess       Reference to the Session structure to be initialized.
     * @param expRuntime Reference to the Experiment structure to be initialized.
     * @param argc       Number of command-line arguments passed to the program.
     * @param argv       Array of command-line argument strings.
     *
     * @note argv[1] is expected to be the IP address, argv[2] the port, argv[4] the energy 
     * detection threshold, and argv[5] the program runtime in seconds.
     */
    sess.UDP_IP = argv[1]; // IP address of data logger or simulator
    if (sess.UDP_IP == "self") {
        sess.UDP_IP = "127.0.0.1";
    }
    sess.UDP_PORT = std::stoi(argv[2]);
    expRuntime.speedOfSound = std::stof(argv[3]);
    expRuntime.energyDetThresh = std::stof(argv[4]);
    expRuntime.programRuntime = std::chrono::seconds(std::stoi(argv[5]));
    std::cout << "Listening to IP address " << sess.UDP_IP << " and port " << sess.UDP_PORT << std::endl;
}

void PrintTimes(const std::span<TimePoint> timestamps)
{
    /**
     * @brief Prints the timestamps provided in the input vector.
     *
     * This function prints the timestamps provided in the input vector
     * in the format "YYYY-MM-DD HH:MM:SS:Microseconds".
     *
     * @param timestamps A vector of TimePoint objects representing the timestamps to be printed.
     *                   TimePoint is a type alias for a time point based on std::chrono::system_clock.
     */

    for (auto &timestamp : timestamps)
    {
        std::time_t timeRepresentation = std::chrono::system_clock::to_time_t(timestamp);
        std::tm timeData = *std::localtime(&timeRepresentation);
        auto microSeconds = std::chrono::duration_cast<std::chrono::microseconds>(timestamp.time_since_epoch()).count() % 1000000;
        std::stringstream msg; // compose message to dispatch
        msg << "Timestamp: "
            << timeData.tm_year + 1900 << '-'
            << timeData.tm_mon + 1 << '-'
            << timeData.tm_mday << ' '
            << timeData.tm_hour << ':'
            << timeData.tm_min << ':'
            << timeData.tm_sec << ':'
            << microSeconds << std::endl;

        std::cout << msg.str();
    }
}

void InitiateOutputFile(std::string &outputFile, std::tm &timeStruct, int64_t microSec, std::string &feature, int NUM_CHAN)
{
    /**
     * @brief Initializes the output file for writing experiment results.
     *
     * This function generates the output file name based on the provided time structure, microsecond timestamp,
     * and feature string. It then opens the file in write mode, clears its contents if it exists, and writes
     * the header for the experiment data. If the file cannot be opened, the function throws a runtime error.
     *
     * @param outputFile Reference to a string where the generated output file name will be stored.
     * @param timeStruct A structure containing the time information used to generate the file name.
     * @param microSec The microsecond part of the timestamp used in the file name.
     * @param feature A string representing the feature to include in the file name.
     * @param NUM_CHAN The number of channels, used to generate the header columns for TDOA and Xcorr.
     * 
     * @throws std::runtime_error if the file cannot be opened for writing.
     */

    outputFile = "deployment_files/" + std::to_string(timeStruct.tm_year + 1900) + '-' + std::to_string(timeStruct.tm_mon + 1) + '-' +
                 std::to_string(timeStruct.tm_mday) + '-' + std::to_string(timeStruct.tm_hour) + '-' + std::to_string(timeStruct.tm_min) + '-' +
                 std::to_string(timeStruct.tm_sec) + '-' + std::to_string(microSec) + '_' + feature;

    std::stringstream msg; // compose message to dispatch
    msg << "created and writting to file: " << outputFile << std::endl;
    std::cout << msg.str();

    // Open the file in write mode and clear its contents if it exists, create a new file otherwise
    std::ofstream file(outputFile, std::ofstream::out | std::ofstream::trunc);
    if (file.is_open())
    {
        file << "usec since Unix Start" << std::setw(20) << "Energy     " << "El.      " << "Az.     ";

        for (int sig = 1; sig < NUM_CHAN; ++sig)
        {
            for (int ref = sig + 1; ref < NUM_CHAN + 1; ++ref)
            {
                file << "TDOA" + std::to_string(sig * 10 + ref) << "  ";
            }
        }

        for (int sig = 1; sig < NUM_CHAN; ++sig)
        {
            for (int ref = sig + 1; ref < NUM_CHAN + 1; ++ref)
            {
                file << "Xcorr" + std::to_string(sig * 10 + ref) << "   ";
            }
        }

        file << std::endl;
        file.close();
    }
    else
    {
        std::stringstream throwMsg; // compose message to dispatch
        throwMsg << "Error: Unable to open file for writing: " << outputFile << std::endl;
        throw std::runtime_error(throwMsg.str());
    }
}

auto ReadFIRFilterFile(const char* fileName) -> std::vector<float>
{
    /**
     * @brief Reads a file containing the FIR filter taps and returns the values as an Armadillo column vector.
     *
     * @param fileName The name (or path) of the file to read.
     *                 The file should contain comma-separated numeric values on each line.
     * @return arma::Col<double> An Armadillo column vector containing the numeric values
     *                           read from the file.
     * @throws std::runtime_error If the file cannot be opened.
     */

    std::ifstream inputFile(fileName);
    if (!inputFile.is_open())
    {
        std::stringstream msg; // compose message to dispatch
        msg << "Error: Unable to open filter file '" << fileName << "'." << std::endl;
        throw std::ios_base::failure(msg.str());
    }
    std::string line;
    std::vector<float> filterValues;
    while (std::getline(inputFile, line))
    {
        // std::vector<float> values;
        std::stringstream stringStream(line);
        std::string token;

        while (std::getline(stringStream, token, ','))
        {
            try
            {
                float value = std::stof(token);
                filterValues.push_back(value);
            }
            catch (const std::invalid_argument &e)
            {
                std::stringstream errMsg; // compose message to dispatch
                errMsg << "Invalid numeric value: " << token << std::endl;
                std::cerr << errMsg.str();
            }
        }
    }
    return filterValues;
}

void ClearQueue(std::queue<std::vector<uint8_t>> &fullQueue)
{
    /**
     * @brief This function effectively clears the given queue by swapping it with an
     * empty queue, thus removing all its elements.
     *
     * @param q The queue to be cleared. This queue holds vectors of uint8_t.
     */

    std::queue<std::vector<uint8_t>> empty;
    std::swap(fullQueue, empty);
}

bool WithProbability(double probability)
{
    /**
     * @brief Generates a boolean value based on the given probability.
     * This fucntion is used for testing.
     *
     * @param probability The probability (between 0.0 and 1.0) of returning true.
     * @return bool Returns true with the specified probability, otherwise returns false.
     */

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    double randomValue = dis(gen);
    return randomValue < probability;
}

Eigen::MatrixXd LoadHydrophonePositions(const char* filename)
{
    /**
     * @brief Loads hydrophone positions from a CSV file and calculates relative positions.
     *
     * @param filename The name (or path) of the CSV file containing the hydrophone positions.
     * @return Eigen::MatrixXd A matrix containing the relative positions between hydrophones.
     * 
     * @throws std::ios_base::failure if the file cannot be opened.
     */
    std::ifstream inputFile(filename);

    if (!inputFile.is_open())
    {
        std::stringstream msg; // compose message to dispatch
        msg << "Error: Unable to open filter file '" << filename << "'." << std::endl;
        throw std::ios_base::failure(msg.str());
    }

    std::cout << "Reading hydrophone positions..." << std::endl;

    std::vector<std::vector<double>> temp_positions;
    std::string line;
    std::string token;

    // First, read the data into a temporary vector of vectors to determine dimensions
    int numRows = 0;
    int numCols = 0;
    while (std::getline(inputFile, line))
    {
        std::stringstream ss(line);
        std::vector<double> row_data;
        while (std::getline(ss, token, ','))
        {
            row_data.push_back(std::stod(token)); // Convert token to double
        }
        if (numCols == 0)
        {
            numCols = row_data.size(); // Set number of columns based on the first row
        }
        temp_positions.push_back(row_data);
        numRows++;
    }

    // Now create the Eigen matrix with determined dimensions
    Eigen::MatrixXd positions(numRows, numCols);

    // Populate the Eigen matrix with the data from the temp_positions vector
    for (int i = 0; i < numRows; ++i)
    {
        for (int j = 0; j < numCols; ++j)
        {
            positions(i, j) = temp_positions[i][j];
        }
    }

    // Print positions for debugging
    // std::cout << "Printing positions: " << std::endl;
    // std::cout << positions << std::endl;

    // Initialize relativePositions with the correct size
    int numRelativePositions = numRows * (numRows - 1) / 2;
    Eigen::MatrixXd relativePositions(numRelativePositions, numCols);

    int index = 0;
    for (int i = 0; i < numRows; i++)
    {
        for (int j = i + 1; j < numRows; j++)
        {
            for (int k = 0; k < numCols; k++)
            {
                relativePositions(index, k) = positions(j, k) - positions(i, k);
            }
            index++;
        }
    }

    return relativePositions;
}

void ShouldFlushBuffer(BufferWriter &bufferWriter, Session &sess, ExperimentRuntime &expRuntime, const std::chrono::system_clock::time_point& startLoop){
    auto elapsedTime = startLoop - expRuntime.programStartTime;
    auto timeSinceLastWrite = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - bufferWriter._lastFlushTime);
    
    bool timeToWrite = bufferWriter._flushInterval <= timeSinceLastWrite;
    bool timeToExit = elapsedTime >= expRuntime.programRuntime;
    bool bufferIsFull = sess.peakTimesBuffer.size() >= bufferWriter._bufferSizeThreshold;
    
    if (bufferIsFull || timeToExit|| timeToWrite)
    {
        std::cout << "Flushing buffers of length: " << sess.peakTimesBuffer.size() << std::endl;
        bufferWriter.write(sess.Buffer, sess.peakTimesBuffer, expRuntime.detectionOutputFile);
        if (timeToExit){
            std::cout << "Terminating program from inside DataProcessor... duration reached \n";
            std::exit(0);
        }
    }
}

void WritePulseAmplitudes(std::span<float> clickPeakAmps, std::span<TimePoint> timestamps, const std::string &filename)
{
    /**
     * @brief Writes pulse amplitudes and corresponding timestamps to a file.
     *
     * @param clickPeakAmps A reference to a vector of doubles containing pulse amplitudes.
     * @param timestamps A reference to a vector of TimePoint objects representing the timestamps corresponding to the pulse amplitudes.
     * @param filename A string specifying the output file path or name.
     */

    // std::cout << "clickPeakAmps.size(): " << clickPeakAmps.size() << std::endl;
    std::ofstream outfile(filename, std::ios::app);
    if (outfile.is_open())
    {
        // Check if vectors have the same size
        if (clickPeakAmps.size() != timestamps.size())
        {
            std::cerr << "Error: Click amplitude and timestamp vectors have different sizes. \n";
            return;
        }

        // Write data rows
        for (size_t i = 0; i < clickPeakAmps.size(); ++i)
        {
            auto time_point = timestamps[i];
            auto time_since_epoch = std::chrono::duration_cast<std::chrono::microseconds>(time_point.time_since_epoch());
            outfile << time_since_epoch.count() << std::setw(20) << clickPeakAmps[i] << std::endl;
        }
    }
    else
    {
        std::stringstream msg; // compose message to dispatch
        msg << "Error: Could not open file " << filename << std::endl;
        std::cerr << msg.str();
    }
    outfile.close();
}

void PrintFirstFiveValues(const Eigen::MatrixXf &savedFFTs, const Eigen::MatrixXf &invFFT)
{
    int numRows = 5; // savedFFTs.rows();
    int numCols = 4; // std::min(5, static_cast<int>(savedFFTs.cols()));  // Ensure we only access up to 5 elements

    std::cout << "First 5 values from each row of savedFFTs:" << std::endl;
    for (int i = 0; i < numRows; ++i)
    {
        std::cout << "Row " << i + 1 << ": ";
        for (int j = 0; j < numCols; ++j)
        {
            std::cout << savedFFTs(i, j) << " ";
        }
        std::cout << std::endl;
    }

    std::cout << "\nFirst 5 values from each row of invFFT:" << std::endl;
    for (int i = 0; i < numRows; ++i)
    {
        std::cout << "Row " << i + 1 << ": ";
        for (int j = 0; j < numCols; ++j)
        {
            std::cout << invFFT(i, j) << " ";
        }
        std::cout << std::endl;
    }
}

void WriteArray(const std::span<Eigen::VectorXf> array, const std::span<TimePoint> timestamps, const std::string &filename)
{
    /**
     * @brief Writes pulse amplitudes and corresponding timestamps to a file.
     *
     * @param array A reference to an Eigen vector of doubles containing pulse amplitudes.
     * @param timestamps A reference to a vector of TimePoint objects representing the timestamps corresponding to the pulse amplitudes.
     * @param filename A string specifying the output file path or name.
     */

    std::ofstream outfile(filename, std::ios::app);
    if (outfile.is_open())
    {
        // Check if vectors have the same size (uncomment if needed)
        /*
        if (array.size() != timestamps.size()) {
            std::cerr << "Error: Click amplitude and timestamp vectors have different sizes." << std::endl;
            return;
        }
        */

        // Write data rows

        for (size_t row = 0; row < array.size(); row++)
        {
            auto time_point = timestamps[row];
            auto time_since_epoch = std::chrono::duration_cast<std::chrono::microseconds>(time_point.time_since_epoch());

            outfile << time_since_epoch.count() << std::setw(20);
            for (int i = 0; i < array[row].size(); ++i)
            {
                outfile << std::fixed << std::setprecision(5) << std::right << array[row][i] << " ";
            }
            outfile << std::endl;
        }
    }
    else
    {
        std::stringstream msg; // compose message to dispatch
        msg << "Error: Could not open file " << filename << std::endl;
        std::cerr << msg.str();
    }
    outfile.close();
}

void WriteDataToCerr(std::span<TimePoint> dataTimes, std::vector<std::vector<uint8_t>> dataBytesSaved)
{
    std::stringstream msg; // compose message to dispatch
    msg << "Timestmaps of data causing error: \n";
    for (const auto timestamp : dataTimes)
    {
        auto convertedTime = std::chrono::duration_cast<std::chrono::microseconds>(timestamp.time_since_epoch()).count();
        msg << convertedTime << std::endl;
    }
    msg << std::endl;

    msg << "Errored bytes of last packets: " << std::endl;
    for (const auto &byteArray : dataBytesSaved)
    {
        msg << std::endl;
        for (const auto data : byteArray)
        {
            msg << std::setw(2) << std::setfill('0') << static_cast<int>(data);
        }
        msg << std::endl;
    }
    msg << std::endl;
    std::cerr << msg.str();
}
