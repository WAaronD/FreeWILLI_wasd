#include "TDOA_estimation.h"

auto GCC_PHAT_FFTW(Eigen::MatrixXcf &savedFFTs, fftwf_plan &inverseFFT,
                   const int interp, int &paddedLength,
                   unsigned int &NUM_CHAN, const unsigned int &SAMPLE_RATE)
    -> std::tuple<Eigen::VectorXf, Eigen::VectorXf>
{
    /**
     * @brief Computes the Generalized Cross-Correlation with Phase Transform (GCC-PHAT) between pairs of signals.
     *
     * @param savedFFTs A reference to an Eigen matrix containing the input signals. Each column represents a signal.
     * @param ip1 An FFTW plan for inverse FFT computations.
     * @param interp An integer specifying the interpolation factor used in the computation.
     * @param fftLength An integer specifying the length of the FFT.
     * @param NUM_CHAN An unsigned integer specifying the number of channels.
     * @param SAMPLE_RATE An unsigned integer specifying the sample rate.
     *
     * @return An Eigen vector of doubles containing the computed TDOA estimates for all unique pairs of signals.
     */

    int fftLength = savedFFTs.col(0).size();
    int numTDOAs = NUM_CHAN * (NUM_CHAN - 1) / 2;
    Eigen::VectorXf tauVector(numTDOAs);
    Eigen::VectorXf XCorrPeaks(numTDOAs);
    Eigen::VectorXcf SIG1(fftLength);
    Eigen::VectorXcf SIG2(fftLength);

    static Eigen::VectorXcf crossSpectraMagnitudeNorm(fftLength);
    static Eigen::VectorXf crossCorr(paddedLength);

    if (inverseFFT == nullptr)
    {
        inverseFFT = fftwf_plan_dft_c2r_1d(paddedLength, reinterpret_cast<fftwf_complex *>(crossSpectraMagnitudeNorm.data()), crossCorr.data(), FFTW_MEASURE);
        crossSpectraMagnitudeNorm.setZero();
        crossCorr.setZero();
    }

    int pairCounter = 0;
    for (unsigned int sig1_ind = 0; sig1_ind < (NUM_CHAN - 1); sig1_ind++)
    {
        for (unsigned int sig2_ind = sig1_ind + 1; sig2_ind < NUM_CHAN; sig2_ind++)
        {
            SIG1 = savedFFTs.col(sig1_ind);
            SIG2 = savedFFTs.col(sig2_ind);

            Eigen::VectorXcf crossSpectra = SIG1.array() * SIG2.conjugate().array();
            Eigen::VectorXf crossSpectraMagnitude = crossSpectra.cwiseAbs();

            if ((crossSpectraMagnitude.array() == 0).any())
            {
                crossSpectraMagnitude = crossSpectraMagnitude.unaryExpr([](float x)
                                                                        { return x == 0 ? 1.0f : x; });
            }

            if ((crossSpectraMagnitude.array().isInf()).any())
            {
                throw GCC_Value_Error("FFTW crossSpectraMagnitude contains inf value");
            }
            else if ((crossSpectraMagnitude.array().isNaN()).any())
            {
                throw GCC_Value_Error("FFTW crossSpectraMagnitude contains nan value");
            }
            crossSpectraMagnitudeNorm = crossSpectra.array() / crossSpectraMagnitude.array();

            fftwf_execute(inverseFFT);

            int maxShift = (interp * (paddedLength / 2));
            Eigen::VectorXf back = crossCorr.tail(maxShift);
            Eigen::VectorXf front = crossCorr.head(maxShift);

            Eigen::VectorXf crossCorrInverted(maxShift * 2);
            crossCorrInverted << back, front;

            Eigen::Index maxIndex;
            float maxValue = crossCorrInverted.maxCoeff(&maxIndex);

            double shift = static_cast<double>(maxIndex) - maxShift;
            double timeDelta = shift / (interp * SAMPLE_RATE);
            tauVector(pairCounter) = timeDelta;
            XCorrPeaks(pairCounter) = maxValue;

            pairCounter++;
        }
    }
    return std::make_tuple(tauVector, XCorrPeaks);
}

Eigen::VectorXf CrossCorr(const Eigen::MatrixXf &channel_matrix, float fs, float max_tau = -1.0f, int interp = 16)
{
    int num_channels = channel_matrix.cols();
    int signal_length = channel_matrix.rows();

    Eigen::MatrixXf tau_matrix = Eigen::MatrixXf::Zero(num_channels, num_channels);
    Eigen::VectorXf tau_vector((num_channels * (num_channels - 1)) / 2);

    int pairing = 0;
    for (int sig_ind = 0; sig_ind < num_channels - 1; ++sig_ind)
    {
        for (int ref_ind = sig_ind + 1; ref_ind < num_channels; ++ref_ind)
        {
            // Get the absolute values of the signals
            Eigen::VectorXf sig = channel_matrix.col(sig_ind).array().abs();
            Eigen::VectorXf refsig = channel_matrix.col(ref_ind).array().abs();

            // Cross-correlation of the two signals
            Eigen::VectorXf cross_corr = Eigen::VectorXf::Zero(2 * signal_length - 1);
            for (int i = 0; i < signal_length; ++i)
            {
                cross_corr.segment(i, signal_length) += sig(i) * refsig;
            }

            // Time axis for the cross-correlation
            Eigen::VectorXf time_axis(2 * signal_length - 1);
            std::iota(time_axis.data(), time_axis.data() + time_axis.size(), -signal_length + 1);
            time_axis /= fs;

            // Find the index of the maximum value in the cross-correlation
            int max_index = 0;
            cross_corr.cwiseAbs().maxCoeff(&max_index);

            // Calculate the TDOA in seconds
            float tdoa = time_axis(max_index);
            tau_matrix(ref_ind, sig_ind) = tdoa;
            tau_vector(pairing) = tdoa;
            pairing++;
        }
    }

    return tau_vector;
}

// Precompute the QR decomposition
Eigen::ColPivHouseholderQR<Eigen::MatrixXd> precomputedQR(const Eigen::MatrixXd &H) {
    return H.colPivHouseholderQr();
}

Eigen::VectorXf TDOA_To_DOA_GeneralArray(const Eigen::ColPivHouseholderQR<Eigen::MatrixXd> &qr, double c, const Eigen::VectorXf &tdoa)
{
    // Convert tdoa to VectorXd
    Eigen::VectorXd tdoa_d = tdoa.cast<double>();

    // Solve for DOA using least squares
    Eigen::VectorXd scaled_tdoa = tdoa_d * c;
    Eigen::VectorXd doa = qr.solve(scaled_tdoa);

    // Normalize the DOA vector
    doa /= doa.norm();

    // Calculate elevation and azimuth
    float el = static_cast<float>(180.0 - std::acos(doa(2)) * 180.0 / M_PI);
    float az = static_cast<float>(std::atan2(doa(1), doa(0)) * 180.0 / M_PI);

    // Create a result vector containing el, az, and the DOA vector components
    Eigen::VectorXf result_vector(2);
    result_vector << el, az;

    return result_vector;
}

Eigen::VectorXf TDOA_To_DOA_VerticalArray(Eigen::VectorXf &TDOAs, const double &soundSpeed, std::span<float> chanSpacing)
{
    /**
     * @brief Estimates the vertical direction of arrival (DOA) using time difference of arrivals (TDOAs)
     * between microphone channels in an array.
     *
     * @param TDOAs (Eigen::VectorXd): Array of time difference of arrivals (TDOAs) between microphone pairs.
     * @param soundSpeed (double): Speed of sound in the medium, in meters per second.
     * @param chanSpacing (Eigen::VectorXi): Array of vertical separation between pairwise microphone channels, in meters.
     *
     * @return Eigen::VectorXd Array containing the estimated vertical DOA angles in degrees.
     */

    Eigen::VectorXf vals = (TDOAs * soundSpeed).array(); // calculate the distance differences and normalize

    // Convert std::vector<double> to Eigen::VectorXd
    Eigen::VectorXf divisorEigen = Eigen::Map<Eigen::VectorXf>(chanSpacing.data(), chanSpacing.size());

    // Perform element-wise division
    vals = vals.array() / divisorEigen.array();
    float max = vals.array().abs().maxCoeff();
    if (max > 1.0f)
    {
        std::cerr << "Out of bounds values: " << std::endl;
        std::cerr << vals.transpose() << std::endl;
        std::cerr << "TDOAs: " << TDOAs.transpose() << std::endl;
        // throw std::runtime_error("Bad TDOA values encountered!");
    }

    // Ensure that values are between -1 and 1 for arcsin
    vals = vals.array().min(1).max(-1);

    // Convert angle to degrees
    return vals.array().acos() * 180.0f / 3.1415f;
}