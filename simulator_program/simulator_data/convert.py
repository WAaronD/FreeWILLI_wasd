import scipy.io as sio
import numpy as np
import h5py
import os

def convert_mat_to_npy_or_hdf5(mat_file, output_dir, format='npy'):
    data = sio.loadmat(mat_file)

    base_filename = os.path.splitext(os.path.basename(mat_file))[0]

    if format == 'npy':
        for key, value in data.items():
            if isinstance(value, np.ndarray):
                # Convert to a specific dtype if necessary
                if value.dtype == 'O':  # Check if dtype is object
                    # Convert objects to a specific numeric type (e.g., float32)
                    value = np.array(value, dtype=np.float32)
                npy_filename = os.path.join(output_dir, f"{base_filename}_{key}.npy")
                np.save(npy_filename, value)
    elif format == 'hdf5':
        h5_filename = os.path.join(output_dir, f"{base_filename}.h5")
        with h5py.File(h5_filename, 'w') as h5f:
            for key, value in data.items():
                if isinstance(value, np.ndarray):
                    if value.dtype == 'O':  # Convert objects to a numeric type
                        value = np.array(value, dtype=np.float32)
                    h5f.create_dataset(key, data=value)

def convert_all_mat_files(input_dir, output_dir, format='npy'):
    os.makedirs(output_dir, exist_ok=True)
    for file_name in os.listdir(input_dir):
        if file_name.endswith('.mat'):
            mat_file = os.path.join(input_dir, file_name)
            convert_mat_to_npy_or_hdf5(mat_file, output_dir, format=format)

# Example usage
input_dir = 'integration_test/'  # Directory containing .mat files
output_dir = 'integration_test/'  # Directory to save the converted files
format = 'npy'  # or 'hdf5'

# Convert all .mat files in the input directory
convert_all_mat_files(input_dir, output_dir, format=format)
