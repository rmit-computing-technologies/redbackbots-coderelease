# Using LoLA on Nao v6 with CMake

## Compilation

Compile using the Linux compile script and target LolaTest

```
./Make/Linux/compile -r Release LolaTest
```

## (OLD) Compilation - SofctBank CTC

Download the toolchain. ;)
Run path-to-toolchain/yocto-sdk/relocate_qitoolchain.sh
!!! After this the toolchain can't be moved or copied to other devices anymore!

Set the correct path in cross-config.cmake (replace path-to-toolchain with the right folder).

Build the example:
```
mkdir build
cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cross-config.cmake
make
```

## Run

Copy ``./Build/Linux/LolaTest/Release/lolatest`` to the robot.

Start ./lolatest on the robot and have fun! :)

## More Information

Check out lola_connector.cpp to see where you can plug in your walking engine.

If you have questions or are interested in any other modules feel free to contact us at naohtwk@gmail.com.
