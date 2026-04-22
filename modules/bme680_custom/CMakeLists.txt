
# include the source files for the BME680 sensor driver and header files for the BME680 sensor driver 

# Add the library to the Zephyr build system
zephyr_library()

# Tell Zephyr where the source code is
zephyr_library_sources(src/bme680_lib.c)

# Make the 'include' folder visible to the whole project
zephyr_include_directories(include)