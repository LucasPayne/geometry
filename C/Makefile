
barycentric_test: barycentric_test.o barycentric.o
	gcc -o barycentric_test barycentric_test.o barycentric.o

barycentric_benchmark: barycentric_benchmark.o barycentric.o
	gcc -o barycentric_benchmark barycentric_benchmark.o barycentric.o

test: barycentric_test
	./barycentric_test

benchmark: barycentric_benchmark
	time --format=%E ./barycentric_benchmark 10000000
