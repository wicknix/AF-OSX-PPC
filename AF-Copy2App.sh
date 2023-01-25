#!/bin/csh -f

# from AuroraFox, modified for TenFourFox, and then for ArcticFox

set verbose
set ppath=$1
#set ppath=$1
cp -RL obj-ff-dbg/dist/ArcticFoxPPC.app $ppath || exit
cd $ppath/Contents/MacOS || exit

# determine which libgcc got linked (default to /opt/local/lib/gcc46)
set libgcc=gcc48
otool -L XUL | grep stdc | grep --silent /libgcc/ && set libgcc=libgcc

ditto /opt/macports-tff/lib/$libgcc/libstdc++.6.dylib ./
ditto /opt/macports-tff/lib/$libgcc/libgcc_s.1.dylib ./
install_name_tool -id @executable_path/libgcc_s.1.dylib libgcc_s.1.dylib
install_name_tool -id @executable_path/libstdc++.6.dylib libstdc++.6.dylib
install_name_tool -change /opt/macports-tff/lib/$libgcc/libgcc_s.1.dylib @executable_path/libgcc_s.1.dylib libstdc++.6.dylib

# fix Firefox and xpcshell
install_name_tool -change /opt/macports-tff/lib/$libgcc/libstdc++.6.dylib @executable_path/libstdc++.6.dylib -change /opt/macports-tff/lib/$libgcc/libgcc_s.1.dylib @executable_path/libgcc_s.1.dylib XUL
install_name_tool -change /opt/macports-tff/lib/$libgcc/libstdc++.6.dylib @executable_path/libstdc++.6.dylib -change /opt/macports-tff/lib/$libgcc/libgcc_s.1.dylib @executable_path/libgcc_s.1.dylib arcticfox
install_name_tool -change /opt/macports-tff/lib/$libgcc/libstdc++.6.dylib @executable_path/libstdc++.6.dylib -change /opt/macports-tff/lib/$libgcc/libgcc_s.1.dylib @executable_path/libgcc_s.1.dylib arcticfox-bin
install_name_tool -change /opt/macports-tff/lib/$libgcc/libstdc++.6.dylib @executable_path/libstdc++.6.dylib -change /opt/macports-tff/lib/$libgcc/libgcc_s.1.dylib @executable_path/libgcc_s.1.dylib libfreebl3.dylib
install_name_tool -change /opt/macports-tff/lib/$libgcc/libstdc++.6.dylib @executable_path/libstdc++.6.dylib -change /opt/macports-tff/lib/$libgcc/libgcc_s.1.dylib @executable_path/libgcc_s.1.dylib libmozalloc.dylib
install_name_tool -change /opt/macports-tff/lib/$libgcc/libstdc++.6.dylib @executable_path/libstdc++.6.dylib -change /opt/macports-tff/lib/$libgcc/libgcc_s.1.dylib @executable_path/libgcc_s.1.dylib libmozglue.dylib
install_name_tool -change /opt/macports-tff/lib/$libgcc/libstdc++.6.dylib @executable_path/libstdc++.6.dylib -change /opt/macports-tff/lib/$libgcc/libgcc_s.1.dylib @executable_path/libgcc_s.1.dylib libnss3.dylib
install_name_tool -change /opt/macports-tff/lib/$libgcc/libstdc++.6.dylib @executable_path/libstdc++.6.dylib -change /opt/macports-tff/lib/$libgcc/libgcc_s.1.dylib @executable_path/libgcc_s.1.dylib libnssckbi.dylib
install_name_tool -change /opt/macports-tff/lib/$libgcc/libstdc++.6.dylib @executable_path/libstdc++.6.dylib -change /opt/macports-tff/lib/$libgcc/libgcc_s.1.dylib @executable_path/libgcc_s.1.dylib libnssdbm3.dylib 
install_name_tool -change /opt/macports-tff/lib/$libgcc/libstdc++.6.dylib @executable_path/libstdc++.6.dylib -change /opt/macports-tff/lib/$libgcc/libgcc_s.1.dylib @executable_path/libgcc_s.1.dylib libplugin_child_interpose.dylib
install_name_tool -change /opt/macports-tff/lib/$libgcc/libstdc++.6.dylib @executable_path/libstdc++.6.dylib -change /opt/macports-tff/lib/$libgcc/libgcc_s.1.dylib @executable_path/libgcc_s.1.dylib libsoftokn3.dylib
install_name_tool -change /opt/macports-tff/lib/$libgcc/libstdc++.6.dylib @executable_path/libstdc++.6.dylib -change /opt/macports-tff/lib/$libgcc/libgcc_s.1.dylib @executable_path/libgcc_s.1.dylib updater.app/Contents/MacOS/updater
install_name_tool -change /opt/macports-tff/lib/$libgcc/libstdc++.6.dylib @executable_path/../MacOS/libstdc++.6.dylib -change /opt/macports-tff/lib/$libgcc/libgcc_s.1.dylib @executable_path/../MacOS/libgcc_s.1.dylib ../Resources/webapprt-stub
install_name_tool -change /opt/macports-tff/lib/$libgcc/libstdc++.6.dylib @executable_path/../../../MacOS/libstdc++.6.dylib -change /opt/macports-tff/lib/$libgcc/libgcc_s.1.dylib @executable_path/../../../MacOS/libgcc_s.1.dylib ../Resources/browser/components/libbrowsercomps.dylib
#install_name_tool -change /opt/local/lib/$libgcc/libstdc++.6.dylib @executable_path/libstdc++.6.dylib -change /opt/local/lib/$libgcc/libgcc_s.1.dylib @executable_path/libgcc_s.1.dylib libssl3.dylib
#install_name_tool -change /opt/local/lib/$libgcc/libstdc++.6.dylib @executable_path/libstdc++.6.dylib -change /opt/local/lib/$libgcc/libgcc_s.1.dylib @executable_path/libgcc_s.1.dylib libxpcom.dylib
#install_name_tool -change /opt/local/lib/$libgcc/libstdc++.6.dylib @executable_path/libstdc++.6.dylib -change /opt/local/lib/$libgcc/libgcc_s.1.dylib @executable_path/libgcc_s.1.dylib libsmime3.dylib
#install_name_tool -change /opt/local/lib/$libgcc/libstdc++.6.dylib @executable_path/libstdc++.6.dylib -change /opt/local/lib/$libgcc/libgcc_s.1.dylib @executable_path/libgcc_s.1.dylib libnssutil3.dylib
#install_name_tool -change /opt/local/lib/$libgcc/libstdc++.6.dylib @executable_path/libstdc++.6.dylib -change /opt/local/lib/$libgcc/libgcc_s.1.dylib @executable_path/libgcc_s.1.dylib libplc4.dylib
#install_name_tool -change /opt/local/lib/$libgcc/libstdc++.6.dylib @executable_path/libstdc++.6.dylib -change /opt/local/lib/$libgcc/libgcc_s.1.dylib @executable_path/libgcc_s.1.dylib libplds4.dylib
#install_name_tool -change /opt/local/lib/$libgcc/libstdc++.6.dylib @executable_path/libstdc++.6.dylib -change /opt/local/lib/$libgcc/libgcc_s.1.dylib @executable_path/libgcc_s.1.dylib components/libalerts.dylib
#install_name_tool -change /opt/local/lib/$libgcc/libstdc++.6.dylib @executable_path/libstdc++.6.dylib -change /opt/local/lib/$libgcc/libgcc_s.1.dylib @executable_path/libgcc_s.1.dylib libmozjs.dylib 
#install_name_tool -change /opt/local/lib/$libgcc/libstdc++.6.dylib @executable_path/libstdc++.6.dylib -change /opt/local/lib/$libgcc/libgcc_s.1.dylib @executable_path/libgcc_s.1.dylib libmozsqlite3.dylib
#install_name_tool -change /opt/local/lib/$libgcc/libstdc++.6.dylib @executable_path/libstdc++.6.dylib -change /opt/local/lib/$libgcc/libgcc_s.1.dylib @executable_path/libgcc_s.1.dylib libnspr4.dylib
#install_name_tool -change /opt/local/lib/$libgcc/libstdc++.6.dylib @executable_path/libstdc++.6.dylib -change /opt/local/lib/$libgcc/libgcc_s.1.dylib @executable_path/libgcc_s.1.dylib plugin-container.app/Contents/MacOS/plugin-container
#install_name_tool -change /opt/local/lib/$libgcc/libstdc++.6.dylib @executable_path/libstdc++.6.dylib -change /opt/local/lib/$libgcc/libgcc_s.1.dylib @executable_path/libgcc_s.1.dylib crashreporter.app/Contents/MacOS/crashreporter
#install_name_tool -change /opt/local/lib/$libgcc/libstdc++.6.dylib @executable_path/libstdc++.6.dylib -change /opt/local/lib/$libgcc/libgcc_s.1.dylib @executable_path/libgcc_s.1.dylib libsoundtouch.dylib

# fix JS
mv ../Resources/js . || exit
install_name_tool -change /opt/macports-tff/lib/$libgcc/libstdc++.6.dylib @executable_path/libstdc++.6.dylib -change /opt/macports-tff/lib/$libgcc/libgcc_s.1.dylib @executable_path/libgcc_s.1.dylib js

echo "(used libraries from /opt/macports-tff/lib/$libgcc)"
