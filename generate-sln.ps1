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

cmake -G "Visual Studio 16 2019" -A X64 -DCMAKE_INSTALL_PREFIX=($x64_install_dir) ..
cmake --build . --config Release --target ALL_BUILD
cmake --build . --config Release --target INSTALL

# copy dependencies over including system dlls
# copy zlib files 
cp ($x64_build_dir + "\zlib\bin\zlib.dll") ($x64_install_dir + "\bin\")
cp ($x64_build_dir+ "\zlib\lib\zlib.lib") ($x64_install_dir + "\lib\")
cp ($x64_build_dir + "\zlib\lib\zlibstatic.lib") ($x64_install_dir + "\lib\")
cd $currentDirectory.Path


# build for arm64
remove-item -path ($currentDirectory.Path + "\build_arm64") -Force -Recurse -ErrorAction SilentlyContinue 
remove-item -path ($currentDirectory.Path + "\install_arm64") -Force -Recurse -ErrorAction SilentlyContinue  
mkdir ($currentDirectory.Path + "\build_arm64")
mkdir ($currentDirectory.Path + "\install_arm64")
$install_prefix=($currentDirectory.Path + "\install_arm64\") 
cd ./build_arm64
echo ($currentDirectory.Path + "\install_arm64\")
# read-host “Press ENTER to continue...”
cmake -G "Visual Studio 16 2019" -A ARM64 -DCMAKE_INSTALL_PREFIX="../install_arm64/" ..
cmake --build . --config Release --target ALL_BUILD
cmake --build . --config Release --target INSTALL
# copy dependencies over including system dlls
# copy zlib files 
cp ($currentDirectory.Path + "\build_arm64\zlib\bin\zlib.dll") ($currentDirectory.Path + "\install_arm64\bin\")
cp ($currentDirectory.Path + "\build_arm64\zlib\lib\zlib.lib") ($currentDirectory.Path + "\install_arm64\lib\")
cp ($currentDirectory.Path + "\build_arm64\zlib\lib\zlibstatic.lib") ($currentDirectory.Path + "\install_arm64\lib\")
cd $currentDirectory.Path