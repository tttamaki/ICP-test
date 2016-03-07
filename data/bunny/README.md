This folder contains the standford bunny data (bun000.ply, bun045.ply)
obtained from http://graphics.stanford.edu/data/3Dscanrep/ .

From these files, bun000mesh.ply and bun045mesh.ply are created by
- reading the ply file by meshlab, and
- exporting the mesh into another ply file by meshlab (select "File"->"Export mesh as" then choose options "Vert: Flags, Normal" and "Face: Flags") so that pcl::io::loadPolygonFile() can read the ply file.
 - Note: pcl::io::loadPolygonFile() can not load the original bun{000,045}.ply files because the face element does not exist.








