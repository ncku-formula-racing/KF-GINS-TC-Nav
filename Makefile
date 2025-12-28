all:
	@mkdir -p build	cd build
	cmake -S . -B build
	cmake --build build

clean:
	rm -rf build
