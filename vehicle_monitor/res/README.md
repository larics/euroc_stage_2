#How to create an octomap

 1. Create the X3D file with Sketchup and blender as explained below.
 2. Run the script x3dToOctovis.sh (You need to install libx3ds and xsltproc)


## Create 3D model files
You can create an OctoMap file from a variety of 3D mesh formats (3DS, VRML, OBJ ...) by first voxelizing them with [binvox](http://www.cs.princeton.edu/~min/binvox). First, obtain the latest version from the author's homepage (the binaries support more mesh formats).

### Converting to a binvox-compatible format 
VRML97 (2.0) data seems to work best in binvox. If your data is already in that format (and metric), skip forward. Google Sketchup for example is not able to export into a working VRML 2.0 file (the units are all in inches). This, and wrong normals often cause problems in voxelization. 

Useful workflows for some 3D editors:

#### Google SketchUp 8 Pro 
 1. Export to a 3DS file. Make sure to set "Units" to "Meters" in the export options.
 2. Open Blender and be sure to delete the little cube which blender adds by default.
 3. Import the 3DS file from step 1 into Blender. Disable the "Size Constraint" scaling option by setting the value to 0 in the import dialog.
  * Blender >= 2.50 only: In the import dialog, set the forward axis to "Y" and the up axis to "Z".
 4. Continue with the instructions for your Blender version below.

#### Google SketchUp 8 Freeware 
 1. The Blender importer cannot deal with components, so you have to break all components first: Mark all objects in the scene by pressing Ctrl+A, right click on an object and select "explode".
 2. Export to a Collada (.dae) file.
 3. Open Blender and be sure to delete the little cube which blender adds by default.
 4. Import the Collada file from step 1 into Blender (Blender <= 2.49: Use the COLLADA 1.4 importer).
 5. Scale the scene around the coordinate system origin by 0.0254 to convert the dimensions from inches to meters.
 6. Continue with the instructions for Blender below.

#### Blender <= 2.49
 1. Export to VRML97 (.wrl) and deactivate the axis swap option in the export dialog.

#### Blender >= 2.50 
 1. Export to X3D Extensible 3D. In the export options, set the forward axis to "Y" and the up axis to "Z".
 2. Convert the X3D file to VRML97 by applying the XML stylesheet available at [[http://www.web3d.org/x3d/content/X3dToVrml97.xslt]] to the X3D file (e.g. using xsltproc command line tool).


### Voxelizing VRML97 in binvox
Run [binvox](http://www.cs.princeton.edu/~min/binvox) on the exported VRML file. Recommended options:
 * -e exact carving gives best results, but results e.g. in hollow walls (no room for compression in the octree)
 * otherwise try a combination of -c and / or -v. This might or might not work for some meshes
 * -fit gives the smallest bounding box fit
 * -d <numvoxels>: Number of voxels for the longest cube dimension. You need to tweak this number so that the final map (after octree creation) has the desired resolution.

The result is a ".binvox" file. You can verify the overall correctness of scale with the output "bounding box: [...]" (should be correct in meters).

### Converting a binvox file to an OcTree 
With binvox2bt you can create a binary (occupied / free, remainder is unknown) octree. The option `--mark-free` will mark all freespace, otherwise it's "unknown". binvox2bt will also output the smallest voxel resolution, so you might have to go back to the previous step to adjust <numvoxels>. Setting the bounding box with `--bb xmin ymin zmin xmax ymax zmax` to the bounding box printed by the binvox tool (extended by a small margin) reduces the size of the resulting binary tree. Verify the resulting .bt file in octovis.

After converting your 3D data to a binvox file, you can create an Octree map from it by running our tool "binvox2bt". 

_(Authors: Stefan Osswald & Armin Hornung)_
