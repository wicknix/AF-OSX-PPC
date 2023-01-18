
Arctic Fox port for PowerPC OS X Tiger and Leopard made with the help of patches from floodgap systems.

This port is not part of mainline Arctic Fox as it uses a slightly older code base.
It is also thousands of commits behind mainline Arctic Fox.
This is more of a "proof of concept" that could turn in to something really nice if people join in to help.

To build:
You will need PowerPC OS X Tiger (10.4.11) or Leopard (10.5.8) and this preconfigured tool kit: 
https://macintoshgarden.org/apps/the-unofficial-tenfourfox-toolkit

Select one of the example mozcfg's and rename it .mozconfig
Run ./mach build
Wait a few hours (118 minutes on a dual core 2.3ghz G5 running Tiger)
After it builds run ./mach run to test your build

Packaging your build:
Run ./AF-Copy2App.sh /Users/YourName/Desktop/ArcticFox.app
Now relink your libraries if you plan to share your build
Run ./AF-Fix-Links.sh /Users/YourName/Desktop/ArcticFox.app

Cleaning up after you're done:
Run ./mach clobber && rm configure

