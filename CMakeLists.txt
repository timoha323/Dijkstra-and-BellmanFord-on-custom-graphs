cmake_minimum_required(VERSION 3.29)
project(laba4)

set(CMAKE_CXX_STANDARD 26)

add_executable(laba4 main.cpp
        DataStructures/DynamicArraySmart.h
        DataStructures/BTree.h
        DataStructures/HashTable.h
        DataStructures/ISet.h
        DataStructures/KeyValue.h
        DataStructures/SparseMatrix.h
        DataStructures/SparseVector.h
        menu.cpp
        DataStructures/Graph.cpp
        tests.cpp
        tests.h)
