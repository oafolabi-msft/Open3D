$currentDirectory=Get-Location
echo $currentDirectory

# visual studio path
$visualStudioRoot="C:\Program Files (x86)\Microsoft Visual Studio\2019\Enterprise\VC\Redist\MSVC\14.*"
$visualStudioRoot=Get-ChildItem -Path $visualStudioRoot -Directory | Sort-Object -Property LastWriteTime | Select-Object -Last 1


# build for x64
$x64_build_dir=$currentDirectory.Path + "\build_x64_Release"
$x64_install_dir=$currentDirectory.Path + "\install_x64_Release" 
remove-item -path ($x64_build_dir) -Force -Recurse -ErrorAction SilentlyContinue 
remove-item -path ($x64_install_dir) -Force -Recurse -ErrorAction SilentlyContinue  
mkdir ($x64_build_dir)
mkdir ($x64_install_dir)
cd $x64_build_dir

cmake -G "Visual Studio 16 2019" -A X64 -DCMAKE_INSTALL_PREFIX="../install_x64_Release/" ..
cmake --build . --config Release --target ALL_BUILD
cmake --build . --config Release --target INSTALL

# copy dependencies over including system dlls
# copy zlib files 
cp ($x64_build_dir + "\*\bin\*.dll") ($x64_install_dir + "\bin\")
cp ($x64_build_dir+ "\*\lib\*.lib") ($x64_install_dir + "\lib\")

# vc redist
$vsPath=($visualStudioRoot.FullName + "\x64\" )
cp ($vsPath + "*\*.dll") ($x64_install_dir + "\bin\")
cd $currentDirectory.Path


# # build for arm64
# $arm64_build_dir=$currentDirectory.Path + "\build_ARM64_Release"
# $arm64_install_dir=$currentDirectory.Path + "\install_ARM64_Release" 
# remove-item -path ($arm64_build_dir) -Force -Recurse -ErrorAction SilentlyContinue 
# remove-item -path ($arm64_install_dir) -Force -Recurse -ErrorAction SilentlyContinue  
# mkdir ($arm64_build_dir)
# mkdir ($arm64_install_dir) 
# cd $arm64_build_dir
# cmake -G "Visual Studio 16 2019" -A ARM64 -DCMAKE_INSTALL_PREFIX="../install_ARM64_Release/" ..
# cmake --build . --config Release --target ALL_BUILD
# cmake --build . --config Release --target INSTALL
# # copy dependencies over including system dlls
# # copy zlib files 
# cp ($arm64_build_dir + "\*\bin\*.dll") ($arm64_install_dir + "\bin\")
# cp ($arm64_build_dir+ "\*\lib\*.lib") ($arm64_install_dir + "\lib\")

# #vc redist 
# $vsPath=($visualStudioRoot.FullName + "\arm64\" )
# cp ($vsPath + "*\*.dll") ($arm64_install_dir + "\bin\")
# cd $currentDirectory.Path

######################################################################################################################
# Do the same for Debug mode
# visual studio path
$visualStudioRoot="C:\Program Files (x86)\Microsoft Visual Studio\2019\Enterprise\VC\Redist\MSVC\14.*"
$visualStudioRoot=Get-ChildItem -Path $visualStudioRoot -Directory | Sort-Object -Property LastWriteTime | Select-Object -Last 1


# build for x64
$x64_build_dir=$currentDirectory.Path + "\build_x64_Debug"
$x64_install_dir=$currentDirectory.Path + "\install_x64_Debug" 
remove-item -path ($x64_build_dir) -Force -Recurse -ErrorAction SilentlyContinue 
remove-item -path ($x64_install_dir) -Force -Recurse -ErrorAction SilentlyContinue  
mkdir ($x64_build_dir)
mkdir ($x64_install_dir)
cd $x64_build_dir

cmake -G "Visual Studio 16 2019" -A X64 -DCMAKE_INSTALL_PREFIX="../install_x64_Debug/" ..
cmake --build . --config Debug --target ALL_BUILD
cmake --build . --config Debug --target INSTALL

# copy dependencies over including system dlls
# copy zlib files 
cp ($x64_build_dir + "\*\bin\*.dll") ($x64_install_dir + "\bin\")
cp ($x64_build_dir+ "\*\lib\*.lib") ($x64_install_dir + "\lib\")

# vc redist
$vsPath=($visualStudioRoot.FullName + "\debug_nonredist\x64\" )
cp ($vsPath + "*\*.dll") ($x64_install_dir + "\bin\")
cd $currentDirectory.Path

# ucrtbased.dll, copy over
cp ("C:\Program Files (x86)\Windows Kits\10\bin\10.0.19041.0\x64\ucrt\*.dll") ($x64_install_dir + "\bin\")


# # build for arm64
# $arm64_build_dir=$currentDirectory.Path + "\build_ARM64_Debug"
# $arm64_install_dir=$currentDirectory.Path + "\install_ARM64_Debug" 
# remove-item -path ($arm64_build_dir) -Force -Recurse -ErrorAction SilentlyContinue 
# remove-item -path ($arm64_install_dir) -Force -Recurse -ErrorAction SilentlyContinue  
# mkdir ($arm64_build_dir)
# mkdir ($arm64_install_dir) 
# cd $arm64_build_dir
# cmake -G "Visual Studio 16 2019" -A ARM64 -DCMAKE_INSTALL_PREFIX="../install_ARM64_Debug/" ..
# cmake --build . --config Debug --target ALL_BUILD
# cmake --build . --config Debug --target INSTALL
# # copy dependencies over including system dlls
# # copy zlib files 
# cp ($arm64_build_dir + "\*\bin\*.dll") ($arm64_install_dir + "\bin\")
# cp ($arm64_build_dir+ "\*\lib\*.lib") ($arm64_install_dir + "\lib\")

# #vc redist 
# $vsPath=($visualStudioRoot.FullName + "\debug_nonredist\arm64\" )
# cp ($vsPath + "*\*.dll") ($arm64_install_dir + "\bin\")
# cd $currentDirectory.Path

# # ucrtbased.dll, copy over
# cp ("C:\Program Files (x86)\Windows Kits\10\bin\10.0.19041.0\arm64\ucrt\*.dll") ($arm64_install_dir + "\bin\")