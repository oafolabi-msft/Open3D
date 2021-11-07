$currentDirectory=Get-Location
echo $currentDirectory

# build for x64
$x64_build_dir=$currentDirectory.Path + "\build_x64"
$x64_install_dir=$currentDirectory.Path + "\install_x64" 
remove-item -path ($x64_build_dir) -Force -Recurse -ErrorAction SilentlyContinue 
remove-item -path ($x64_install_dir) -Force -Recurse -ErrorAction SilentlyContinue  
mkdir ($x64_build_dir)
mkdir ($x64_install_dir)
cd $x64_build_dir

cmake -G "Visual Studio 16 2019" -A X64 -DCMAKE_INSTALL_PREFIX="../install_x64/" ..
cmake --build . --config Release --target ALL_BUILD
cmake --build . --config Release --target INSTALL

# copy dependencies over including system dlls
# copy zlib files 
cp ($x64_build_dir + "\zlib\bin\zlib.dll") ($x64_install_dir + "\bin\")
cp ($x64_build_dir+ "\zlib\lib\zlib.lib") ($x64_install_dir + "\lib\")
cp ($x64_build_dir + "\zlib\lib\zlibstatic.lib") ($x64_install_dir + "\lib\")
cd $currentDirectory.Path


# build for arm64
$arm64_build_dir=$currentDirectory.Path + "\build_ARM64"
$arm64_install_dir=$currentDirectory.Path + "\install_ARM64" 
remove-item -path ($arm64_build_dir) -Force -Recurse -ErrorAction SilentlyContinue 
remove-item -path ($arm64_install_dir) -Force -Recurse -ErrorAction SilentlyContinue  
mkdir ($arm64_build_dir)
mkdir ($arm64_install_dir) 
cd $arm64_build_dir
cmake -G "Visual Studio 16 2019" -A ARM64 -DCMAKE_INSTALL_PREFIX="../install_ARM64/" ..
cmake --build . --config Release --target ALL_BUILD
cmake --build . --config Release --target INSTALL
# copy dependencies over including system dlls
# copy zlib files 
cp ($arm64_build_dir + "\zlib\bin\zlib.dll") ($arm64_install_dir + "\bin\")
cp ($arm64_build_dir+ "\zlib\lib\zlib.lib") ($arm64_install_dir + "\lib\")
cp ($arm64_build_dir + "\zlib\lib\zlibstatic.lib") ($arm64_install_dir + "\lib\")
cd $currentDirectory.Path