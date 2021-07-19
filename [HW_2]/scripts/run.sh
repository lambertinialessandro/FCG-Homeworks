./bin/ypathtrace tests/01_cornellbox/cornellbox.json -o out/naive/01_cornellbox_512_256.jpg -t naive -s 256 -r 512
./bin/ypathtrace tests/02_matte/matte.json -o out/naive/02_matte_720_256.jpg -t naive -s 256 -r 720
./bin/ypathtrace tests/03_metal/metal.json -o out/naive/03_metal_720_256.jpg -t naive -s 256 -r 720
./bin/ypathtrace tests/04_plastic/plastic.json -o out/naive/04_plastic_720_256.jpg -t naive -s 256 -r 720
./bin/ypathtrace tests/05_glass/glass.json -o out/naive/05_glass_720_256.jpg -t naive -s 256 -r 720
./bin/ypathtrace tests/06_opacity/opacity.json -o out/naive/06_opacity_720_256.jpg -t naive -s 256 -r 720
./bin/ypathtrace tests/07_hair/hair.json -o out/naive/07_hair_720_256.jpg -t naive -s 256 -r 720
./bin/ypathtrace tests/08_lens/lens.json -o out/naive/08_lens_720_256.jpg -t naive -s 256 -r 720

./bin/ypathtrace tests/01_cornellbox/cornellbox.json -o out/path/01_cornellbox_512_256.jpg -t path -s 256 -r 512
./bin/ypathtrace tests/02_matte/matte.json -o out/path/02_matte_720_256.jpg -t path -s 256 -r 720
./bin/ypathtrace tests/03_metal/metal.json -o out/path/03_metal_720_256.jpg -t path -s 256 -r 720
./bin/ypathtrace tests/04_plastic/plastic.json -o out/path/04_plastic_720_256.jpg -t path -s 256 -r 720
./bin/ypathtrace tests/05_glass/glass.json -o out/path/05_glass_720_256.jpg -t path -s 256 -r 720
./bin/ypathtrace tests/06_opacity/opacity.json -o out/path/06_opacity_720_256.jpg -t path -s 256 -r 720
./bin/ypathtrace tests/07_hair/hair.json -o out/path/07_hair_720_256.jpg -t path -s 256 -r 720
./bin/ypathtrace tests/08_lens/lens.json -o out/path/08_lens_720_256.jpg -t path -s 256 -r 720

./bin/ypathtrace tests/11_bathroom1/bathroom1.json -o out/path/11_bathroom1_720_256.jpg -t path -s 256 -r 720
./bin/ypathtrace tests/12_ecosys/ecosys.json -o out/path/12_ecosys_720_256.jpg -t path -s 256 -r 720
./bin/ypathtrace tests/13_bedroom/bedroom.json -o out/path/13_bedroom_720_256.jpg -t path -s 256 -r 720
./bin/ypathtrace tests/14_car1/car1.json -o out/path/14_car1_720_256.jpg -t path -s 256 -r 720
./bin/ypathtrace tests/15_classroom/classroom.json -o out/path/15_classroom_720_256.jpg -t path -s 256 -r 720
./bin/ypathtrace tests/16_coffee/coffee.json -o out/path/16_coffee_720_256.jpg -t path -s 256 -r 720
./bin/ypathtrace tests/17_kitchen/kitchen.json -o out/path/17_kitchen_720_256.jpg -t path -s 256 -r 720

####

./bin/ypathtrace tests/05_glass/refraction.json -o out/path/05_refraction_720_256.jpg -t path -s 256 -r 720

####

./bin/ypathtrace tests/x1_rungholt/rungholt.json -o out/large_scenes/01_rungholt_1280_1024.jpg -t path -s 1024 -r 1280
./bin/ypathtrace tests/x2_bistrointerior/bistrointerior.json -o out/large_scenes/02_bistrointerior_1280_1024.jpg -t path -s 1024 -r 1280
./bin/ypathtrace tests/x3_sanmiguel/sanmiguel.json -o out/large_scenes/03_sanmiguel_1280_1024.jpg -t path -s 1024 -r 1280
./bin/ypathtrace tests/x4_bistroexterior/bistroexterior.json -o out/large_scenes/04_bistroexterior_1280_1024.jpg -t path -s 1024 -r 1280
./bin/ypathtrace tests/x5_landscape/landscape.json -o out/large_scenes/05_landscape_1280_1024.jpg -t path -s 1024 -r 1280

####

./bin/ypathtrace tests/y1_myscene/myscene.json -o out/my_scene/01_myscene_1024_256.jpg -t path -s 256 -r 1024
./bin/ypathtrace tests/y2_myscene/myscene.json -o out/my_scene/02_myscene_1024_256.jpg -t path -s 256 -r 1024
./bin/ypathtrace tests/y3_myscene/myscene.json -o out/my_scene/03_myscene_1024_256.jpg -t path -s 256 -r 1024
./bin/ypathtrace tests/y4_myscene/myscene.json -o out/my_scene/04_myscene_1024_256.jpg -t path -s 256 -r 1024
./bin/ypathtrace tests/y5_myscene/myscene.json -o out/my_scene/05_myscene_1024_256.jpg -t path -s 256 -r 1024

####

./bin/ypathtrace tests/02_matte/matte.json -o out/stratified/02_matte_720_256.jpg -t stratified -s 256 -r 720
./bin/ypathtrace tests/07_hair/hair.json -o out/stratified/07_hair_720_256.jpg -t stratified -s 256 -r 720
./bin/ypathtrace tests/08_lens/lens.json -o out/stratified/08_lens_720_256.jpg -t stratified -s 256 -r 720
