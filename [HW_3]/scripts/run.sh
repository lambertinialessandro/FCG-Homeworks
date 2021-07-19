


./bin/ypathtrace tests/01_surface/surface.json -o out/lowres/01_surface_720_256.jpg -t volpath -s 256 -r 720
./bin/ypathtrace tests/02_rollingteapot/rollingteapot.json -o out/lowres/02_rollingteapot_720_256.jpg -t volpath -s 256 -r 720
./bin/ypathtrace tests/03_volume/volume.json -o out/lowres/03_volume_720_256.jpg -t volpath -s 256 -r 720
./bin/ypathtrace tests/04_head1/head1.json -o out/lowres/04_head1_720_256.jpg -t volpath -s 256 -r 720
./bin/ypathtrace tests/05_head1ss/head1ss.json -o out/lowres/05_head1ss_720_256.jpg -t volpath -s 256 -r 720
./bin/ypathtrace tests/06_easter_egg/easter_egg.json -o out/lowres/06_easter_egg_720_256.jpg -t volpath -s 256 -r 720

######
# RAY-PATCH
######

./bin/ypathtrace tests/01_surface/surface.json -o out/ray_patch/01_surface_720_512.jpg -t volpath -rp -s 512 -r 720
./bin/ypathtrace tests/02_rollingteapot/rollingteapot.json -o out/ray_patch/02_rollingteapot_720_512.jpg -t volpath -rp -s 512 -r 720
./bin/ypathtrace tests/03_volume/volume.json -o out/ray_patch/03_volume_720_512.jpg -t volpath -rp -s 512 -r 720
./bin/ypathtrace tests/04_head1/head1.json -o out/ray_patch/04_head1_720_512.jpg -t volpath -rp -s 512 -r 720
./bin/ypathtrace tests/05_head1ss/head1ss.json -o out/ray_patch/05_head1ss_720_512.jpg -t volpath -rp -s 512 -r 720
./bin/ypathtrace tests/06_easter_egg/easter_egg.json -o out/ray_patch/06_easter_egg_720_256.jpg -t volpath -rp -s 512 -r 720

######
# ADAPTIVE RENDERING
######

# -q 1
./bin/yscenetrace_adp tests/01_surface/surface.json -o out/adaptive/01_surface_720_1.jpg -t volpath -q 1 -r 720
./bin/yscenetrace_adp tests/02_rollingteapot/rollingteapot.json -o out/adaptive/02_rollingteapot_720_1.jpg -t volpath -q 1 -r 720
./bin/yscenetrace_adp tests/03_volume/volume.json -o out/adaptive/03_volume_720_1.jpg -t volpath -q 1 -r 720
./bin/yscenetrace_adp tests/04_head1/head1.json -o out/adaptive/04_head1_720_1.jpg -t volpath -q 1 -r 720
./bin/yscenetrace_adp tests/05_head1ss/head1ss.json -o out/adaptive/05_head1ss_720_1.jpg -t volpath -q 1 -r 720
./bin/yscenetrace_adp tests/06_easter_egg/easter_egg.json -o out/adaptive/06_easter_egg_720_1.jpg -t volpath -q 1 -r 720

# -q 2
./bin/yscenetrace_adp tests/01_surface/surface.json -o out/adaptive/01_surface_720_2.jpg -t volpath -q 2 -r 720
./bin/yscenetrace_adp tests/02_rollingteapot/rollingteapot.json -o out/adaptive/02_rollingteapot_720_2.jpg -t volpath -q 2 -r 720
./bin/yscenetrace_adp tests/03_volume/volume.json -o out/adaptive/03_volume_720_2.jpg -t volpath -q 2 -r 720
./bin/yscenetrace_adp tests/04_head1/head1.json -o out/adaptive/04_head1_720_2.jpg -t volpath -q 2 -r 720
./bin/yscenetrace_adp tests/05_head1ss/head1ss.json -o out/adaptive/05_head1ss_720_2.jpg -t volpath -q 2 -r 720
./bin/yscenetrace_adp tests/06_easter_egg/easter_egg.json -o out/adaptive/06_easter_egg_720_2.jpg -t volpath -q 2 -r 720

# -q 3
./bin/yscenetrace_adp tests/01_surface/surface.json -o out/adaptive/01_surface_720_3.jpg -t volpath -q 3 -r 720
./bin/yscenetrace_adp tests/02_rollingteapot/rollingteapot.json -o out/adaptive/02_rollingteapot_720_3.jpg -t volpath -q 3 -r 720
./bin/yscenetrace_adp tests/03_volume/volume.json -o out/adaptive/03_volume_720_3.jpg -t volpath -q 3 -r 720
./bin/yscenetrace_adp tests/04_head1/head1.json -o out/adaptive/04_head1_720_3.jpg -t volpath -q 3 -r 720
./bin/yscenetrace_adp tests/05_head1ss/head1ss.json -o out/adaptive/05_head1ss_720_3.jpg -t volpath -q 3 -r 720
./bin/yscenetrace_adp tests/06_easter_egg/easter_egg.json -o out/adaptive/06_easter_egg_720_3.jpg -t volpath -q 3 -r 720
