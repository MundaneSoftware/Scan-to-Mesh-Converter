import open3d as o3d
import numpy as np
import laspy
import pye57
import os
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from tqdm import tqdm

def extract_points_from_las(file_path, progress_callback=None):
    """
    Extracts 3D point data from a LAS/LAZ file in batches.

    Args:
        file_path (str): Path to the input LAS/LAZ file.
        progress_callback (callable, optional): A callback function to update progress.

    Returns:
        np.ndarray: A numpy array of 3D points.
    """
    all_points = []

    print(f"Reading LAS/LAZ file: {file_path}")
    with laspy.open(file_path) as las_file:
        total_points = las_file.header.point_count
        batch_size = 1_000_000
        for start_index in tqdm(range(0, total_points, batch_size), desc="Extracting points"):
            end_index = min(start_index + batch_size, total_points)
            points = las_file.read_points(start=start_index, end=end_index)
            coords = np.vstack((points.x, points.y, points.z)).T
            all_points.append(coords)

            if progress_callback:
                progress_callback(int((end_index / total_points) * 100))

    all_points = np.vstack(all_points).astype(np.float64)
    return all_points

def extract_points_from_e57(file_path, progress_callback=None):
    """
    Extract points from an E57 file.

    Args:
        file_path (str): Path to the E57 file.
        progress_callback (callable, optional): Function to report progress as percentage.

    Returns:
        np.ndarray: Extracted 3D points as a NumPy array.
    """
    all_points = []
    with pye57.E57(file_path) as e57_file:
        scan_count = e57_file.scan_count
        for scan_index in tqdm(range(scan_count), desc="Extracting points"):
            scan = e57_file.read_scan(scan_index)
            points = np.vstack((scan['cartesianX'], scan['cartesianY'], scan['cartesianZ'])).T
            all_points.append(points)
            # Update the progress bar
            if progress_callback:
                progress_callback(int((scan_index + 1) / scan_count * 100))
    all_points = np.vstack(all_points).astype(np.float64)
    return all_points

def generate_mesh_with_poisson(points, depth=8, voxel_size=0.01, density_quantile=0.1, smooth_iterations=5, progress_callback=None):
    """
    Generate a 3D mesh using Poisson surface reconstruction.

    Args:
        points (np.ndarray): 3D points for the mesh.
        depth (int): Depth parameter for Poisson reconstruction.
        voxel_size (float): Voxel size for point cloud preprocessing.
        density_quantile (float): Quantile for filtering low-density vertices.
        smooth_iterations (int): Number of smoothing iterations for the mesh.
        progress_callback (callable, optional): Function to report progress as percentage.

    Returns:
        o3d.geometry.TriangleMesh: Generated 3D mesh.
    """
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)

    # Convert points to Open3D PointCloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # Estimate normals for better reconstruction
    print("Estimating normals...")
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=5.0, max_nn=250))

    # Perform Poisson surface reconstruction
    print("Performing Poisson surface reconstruction...")
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=depth)

    # Remove low-density vertices based on the density quantile
    print("Cleaning up low-density vertices...")
    densities = np.asarray(densities)
    vertices_to_remove = densities < np.quantile(densities, density_quantile)
    mesh.remove_vertices_by_mask(vertices_to_remove)

    # Smooth the mesh if requested
    if smooth_iterations > 0:
        print(f"Smoothing the mesh with {smooth_iterations} iterations...")
        mesh = mesh.filter_smooth_simple(number_of_iterations=smooth_iterations)

    # Report progress completion
    if progress_callback:
        progress_callback(100)

    return mesh

def decimate_mesh(mesh, target_triangle_count, progress_callback=None):
    """
    Decimate a 3D mesh to reduce the number of triangles.

    Args:
        mesh (o3d.geometry.TriangleMesh): The input 3D mesh.
        target_triangle_count (int): Target number of triangles for decimation.
        progress_callback (callable, optional): Function to report progress as percentage.

    Returns:
        o3d.geometry.TriangleMesh: Decimated 3D mesh.
    """
    print(f"Decimating the mesh to {target_triangle_count} triangles...")
    decimated_mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=target_triangle_count)
    print(f"Decimated mesh has {len(decimated_mesh.triangles)} triangles.")

    # Report progress completion
    if progress_callback:
        progress_callback(100)

    return decimated_mesh

def write_obj(file_path, mesh):
    """
    Write a 3D mesh to an OBJ file.

    Args:
        file_path (str): Path to save the OBJ file.
        mesh (o3d.geometry.TriangleMesh): 3D mesh to save.
    """
    o3d.io.write_triangle_mesh(file_path, mesh)
    print(f"Mesh written to {file_path}")

def convert_to_obj(input_path, obj_path, file_type, depth=8, voxel_size=0.01, density_quantile=0.1, smooth_iterations=5, target_triangle_count=100000, progress_callback=None):
    """
    Convert an E57, LAS, or LAZ file to an OBJ file with mesh generation.

    Args:
        input_path (str): Path to the input file.
        obj_path (str): Path to the output OBJ file.
        file_type (str): Type of the input file ("e57", "las", "laz").
        depth (int): Depth parameter for Poisson reconstruction.
        voxel_size (float): Voxel size for point cloud preprocessing.
        density_quantile (float): Quantile for filtering low-density vertices.
        smooth_iterations (int): Number of smoothing iterations for the mesh.
        target_triangle_count (int): Target number of triangles for decimation.
        progress_callback (callable, optional): Function to report progress and stage.
    """
    # Extract points based on file type
    if progress_callback:
        progress_callback("Extracting points", 0)

    if file_type in ["las", "laz"]:
        points = extract_points_from_las(input_path, progress_callback=lambda p: progress_callback("Extracting points", p))
    elif file_type == "e57":
        points = extract_points_from_e57(input_path, progress_callback=lambda p: progress_callback("Extracting points", p))
    else:
        raise ValueError("Unsupported file type")

    # Generate the 3D mesh from points
    if progress_callback:
        progress_callback("Generating mesh", 0)
    mesh = generate_mesh_with_poisson(points, depth=depth, voxel_size=voxel_size, density_quantile=density_quantile, smooth_iterations=smooth_iterations, progress_callback=lambda p: progress_callback("Generating mesh", p))

    # Decimate the mesh to reduce complexity
    if progress_callback:
        progress_callback("Decimating mesh", 0)
    mesh = decimate_mesh(mesh, target_triangle_count, progress_callback=lambda p: progress_callback("Decimating mesh", p))

    # Write the final mesh to an OBJ file
    write_obj(obj_path, mesh)

def run_conversion():
    """
    Run the conversion process from the GUI inputs.
    """
    input_path = entry_input.get()
    obj_path = entry_obj.get()
    file_type = combo_file_type.get().lower()
    depth = int(entry_depth.get())
    voxel_size = float(entry_voxel_size.get())
    density_quantile = float(entry_density_quantile.get())
    smooth_iterations = int(entry_smooth_iterations.get())
    target_triangle_count = int(entry_target_triangle_count.get())

    # Validate input paths
    if not os.path.isfile(input_path):
        messagebox.showerror("Error", "Please select a valid input file.")
        return
    if not obj_path:
        messagebox.showerror("Error", "Please specify an output path.")
        return

    def progress_callback(stage, value):
        """Update the GUI with progress."""
        label_status.config(text=f"{stage}: {value}%")
        progress_bar['value'] = value
        root.update_idletasks()

    # Perform the conversion
    try:
        convert_to_obj(input_path, obj_path, file_type, depth, voxel_size, density_quantile, smooth_iterations, target_triangle_count, progress_callback)
        messagebox.showinfo("Success", f"Mesh successfully saved to {obj_path}")
    except ValueError as e:
        messagebox.showerror("Error", str(e))

# Tkinter GUI setup
root = tk.Tk()
root.title("File to OBJ Converter")

# GUI layout and input fields
tk.Label(root, text="Input File Path").grid(row=0, column=0, sticky="w")
entry_input = tk.Entry(root, width=50)
entry_input.grid(row=0, column=1)

def select_input_file():
    """Open file dialog to select an input file."""
    file_path = filedialog.askopenfilename(filetypes=[("Supported files", "*.e57 *.las *.laz")])
    entry_input.delete(0, tk.END)
    entry_input.insert(0, file_path)

tk.Button(root, text="Browse", command=select_input_file).grid(row=0, column=2)

file_types = ["E57", "LAS", "LAZ"]
combo_file_type = ttk.Combobox(root, values=file_types, state="readonly")
combo_file_type.set("E57")
combo_file_type.grid(row=1, column=1)

tk.Label(root, text="OBJ Output Path").grid(row=2, column=0, sticky="w")
entry_obj = tk.Entry(root, width=50)
entry_obj.grid(row=2, column=1)

def select_output_path():
    """Open file dialog to select a save path for the OBJ file."""
    file_path = filedialog.asksaveasfilename(defaultextension=".obj", filetypes=[("OBJ files", "*.obj")])
    entry_obj.delete(0, tk.END)
    entry_obj.insert(0, file_path)

tk.Button(root, text="Browse", command=select_output_path).grid(row=2, column=2)

tk.Label(root, text="Depth").grid(row=3, column=0, sticky="w")
entry_depth = tk.Entry(root)
entry_depth.insert(0, "15")  # Default value for depth
entry_depth.grid(row=3, column=1)

tk.Label(root, text="Voxel Size").grid(row=4, column=0, sticky="w")
entry_voxel_size = tk.Entry(root)
entry_voxel_size.insert(0, "0.01")  # Default value for voxel size
entry_voxel_size.grid(row=4, column=1)

tk.Label(root, text="Density Quantile").grid(row=5, column=0, sticky="w")
entry_density_quantile = tk.Entry(root)
entry_density_quantile.insert(0, "0.05")  # Default value for density quantile
entry_density_quantile.grid(row=5, column=1)

tk.Label(root, text="Smooth Iterations").grid(row=6, column=0, sticky="w")
entry_smooth_iterations = tk.Entry(root)
entry_smooth_iterations.insert(0, "5")  # Default value for smoothing iterations
entry_smooth_iterations.grid(row=6, column=1)

tk.Label(root, text="Target Triangle Count").grid(row=7, column=0, sticky="w")
entry_target_triangle_count = tk.Entry(root)
entry_target_triangle_count.insert(0, "4000000")  # Default value for target triangle count
entry_target_triangle_count.grid(row=7, column=1)

# Progress bar and status label
progress_bar = ttk.Progressbar(root, orient="horizontal", length=400, mode="determinate")
progress_bar.grid(row=9, column=0, columnspan=2, pady=10)

label_status = tk.Label(root, text="Status: Waiting for input")
label_status.grid(row=10, column=0, columnspan=3)

tk.Button(root, text="Convert", command=run_conversion).grid(row=11, column=1, pady=10)

root.mainloop()
