
#### Dependencies

Install cmake if you do not already have it. If there is not already a folder
called `gtest` in this directory, but just a file with this name, you will need
to populate it:

```
cd <top-level directory of git repository>
git submodule update --init --recursive
```
This will download the `gtest` source code, which I use for the library unit
tests.

#### Running the tests

```
./test
```

#### TODO 
- edit `CMakeLists.txt` to take advantage of ExternalProject git submodule 
  features, and include instructions here if necessary
- setup "continuous integration" testing with Travis CI
- setup gtest submodule correctly after copying from my template
- include potentially pulling in `gtest` submodule in `./test` rather than in
  `CMakeLists.txt`
