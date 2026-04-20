
# include the source files for the BME680 sensor driver and header files for the BME680 sensor driver 

zephyr_library()
zephyr_library_sources(src/bme680_lib.c)
zephyr_include_directories(include)