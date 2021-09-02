

oilpaint: src/main.cu build/stb_image.h build/stb_image_write.h
	nvcc src/main.cu -Ibuild -o $@

build:
	mkdir -p $@


build/stb_image_write.h: | build
	wget https://raw.githubusercontent.com/nothings/stb/master/stb_image_write.h -O $@

build/stb_image.h: | build
	wget https://raw.githubusercontent.com/nothings/stb/master/stb_image.h -O $@
