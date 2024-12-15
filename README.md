# LAS/E57 to OBJ Converter

This Python project converts LAS, LAZ, and E57 files into OBJ format by generating a 3D mesh using Poisson surface reconstruction. It provides a graphical user interface (GUI) for ease of use and supports parameters for fine-tuning the conversion process.

## Features

- Supports input formats: E57, LAS, and LAZ.
- Poisson surface reconstruction for generating 3D meshes.
- Options for smoothing, density filtering, and decimation.
- GUI-based interface for file selection and configuration.

---

## Setup


### 1. Setting up the Environment
Create a virtual environment to manage dependencies:

```bash
python -m venv venv
source venv/bin/activate    # On Windows: venv\Scripts\activate
```

Install the dependencies in the virtual environment:

```bash
pip install open3d-python numpy laspy[laszip] pye57 tqdm
```

---

## Running the Project

### Running from Python Script
Run the script directly to launch the GUI for file conversion:

```bash
python src\scan_to_mesh.py
```

1. Use the **Browse** button to select the input file.
2. Choose the output file path for the OBJ.
3. Configure optional parameters (e.g., depth, density quantile).
4. Click **Convert** to start the process.

---

### Creating an Executable with PyInstaller
To create a standalone executable, use PyInstaller:

1. Install PyInstaller:

```bash
pip install pyinstaller
```

2. Create the executable:

```bash
pyinstaller --onefile --noconsole src\scan_to_mesh.py
```

This generates a standalone executable in the `dist` directory.

3. Run the executable:

```bash
./dist/scan_to_mesh
```

On Windows, use `dist\scan_to_mesh.exe`.

---

## Notes

1. **LAZ Support**: Ensure `laszip` is installed for processing LAZ files (included with `laspy[laszip]`).
2. **Tkinter**: Tkinter comes pre-installed with Python. If it is missing, install it via your system's package manager (e.g., `sudo apt-get install python3-tk` on Ubuntu).

---

## License
This project is open source and available under the MIT License.

## Acknowledgements
- [Open3D](http://www.open3d.org/) for 3D geometry processing.
- [Laspy](https://laspy.readthedocs.io/) for LAS/LAZ file handling.
- [pye57](https://pye57.readthedocs.io/) for E57 file processing.

