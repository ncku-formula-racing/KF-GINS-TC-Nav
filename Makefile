all:
	@mkdir -p build	
	cmake -S . -B build
	cmake --build build

clean:
	rm -rf build
