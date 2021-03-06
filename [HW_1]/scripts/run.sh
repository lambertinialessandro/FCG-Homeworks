./bin/yraytrace tests/02_matte/matte.json -o out/lowres/0x_color_720_9.jpg -s 9 -t color -r 720
./bin/yraytrace tests/02_matte/matte.json -o out/lowres/0x_normal_720_9.jpg -s 9 -t normal -r 720
./bin/yraytrace tests/02_matte/matte.json -o out/lowres/0x_texcoord_720_9.jpg -s 9 -t texcoord -r 720
./bin/yraytrace tests/02_matte/matte.json -o out/lowres/0x_eyelight_720_9.jpg -s 9 -t eyelight -r 720

./bin/yraytrace tests/01_cornellbox/cornellbox.json -o out/lowres/01_cornellbox_512_256.jpg -s 256 -r 512
./bin/yraytrace tests/02_matte/matte.json -o out/lowres/02_matte_720_256.jpg -s 25 -r 720
./bin/yraytrace tests/03_texture/texture.json -o out/lowres/03_texture_720_256.jpg -s 256 -r 720
./bin/yraytrace tests/04_envlight/envlight.json -o out/lowres/04_envlight_720_256.jpg -s 256 -r 720
./bin/yraytrace tests/05_arealight/arealight.json -o out/lowres/05_arealight_720_256.jpg -s 256 -r 720
./bin/yraytrace tests/06_metal/metal.json -o out/lowres/06_metal_720_256.jpg -s 256 -r 720
./bin/yraytrace tests/07_plastic/plastic.json -o out/lowres/07_plastic_720_256.jpg -s 256 -r 720
./bin/yraytrace tests/08_glass/glass.json -o out/lowres/08_glass_720_256.jpg -s 256 -b 8 -r 720
./bin/yraytrace tests/09_opacity/opacity.json -o out/lowres/09_opacity_720_256.jpg -s 256 -r 720
./bin/yraytrace tests/10_hair/hair.json -o out/lowres/10_hair_720_256.jpg -s 256 -r 720
./bin/yraytrace tests/11_bathroom1/bathroom1.json -o out/lowres/11_bathroom1_720_256.jpg -s 256 -b 8 -r 720
./bin/yraytrace tests/12_ecosys/ecosys.json -o out/lowres/12_ecosys_720_256.jpg -s 256 -r 720

# REFRACTION SHADER 
./bin/yraytrace tests/08_glass/glass.json -o out/lowres/13_refraction_720_256.jpg -s 256 -b 8 -t refraction -r 720

# "FLUFFY LIKE" SHADER 
./bin/yraytrace tests/05_arealight/arealight.json -o out/lowres/.00x_try/.01x_try_720.jpg -s 256 -t my1 -r 720
./bin/yraytrace tests/08_glass/glass.json -o out/lowres/.00x_try/.02x_try_720.jpg -s 256 -t my1 -r 720
./bin/yraytrace tests/10_hair/hair.json -o out/lowres/.00x_try/.03x_try_720.jpg -s 256 -t my1 -r 720

# "BI-DIRECTIONAL COLOR (??)" SHADER 
./bin/yraytrace tests/02_matte_test/matte.json -o out/lowres/.00x_try/.05x_try_720.jpg -s 9 -t my2 -r 720

# "TOON LIKE" SHADER 
./bin/yraytrace tests/02_matte/matte.json -o out/lowres/.00x_try/.07x_try_720.jpg -s 9 -t my3 -r 720
./bin/yraytrace tests/03_texture/texture.json -o out/lowres/.00x_try/.08x_try_720.jpg -s 9 -t my3 -r 720

# "HORROR" SHADER 
./bin/yraytrace tests/05_arealight/arealight.json -o out/lowres/.00x_try/.10x_try_720.jpg -s 256 -t my4 -r 720
./bin/yraytrace tests/05_arealight_test/arealight.json -o out/lowres/.00x_try/.11x_try_720.jpg -s 256 -t my4 -r 720
