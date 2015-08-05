#!/bin/bash

if [ "$#" -ne 2 ]; then
  echo "Usage:" >&2
  echo " - $0 X3d file name with extension" >&2
  echo " - $2 Number of voxels for the longest cube dimension" >&2
  exit 1
fi

# Get file name
filename=$(basename "$1")
extension="${filename##*.}"
filename="${filename%.*}"

# Transform to VRML97
xsltproc X3dToVrml97.xslt $filename.x3d > $filename.wrl

# Transform to Octovis
./binvox -e -fit -d $2 $filename.wrl

# Transform to octomap
binvox2bt --mark-free $filename.binvox -o $filename.bt
