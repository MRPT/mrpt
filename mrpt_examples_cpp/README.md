# How to create new C++ examples

This directory contains a collection of example applications for the **Mobile Robot Programming Toolkit (MRPT)**. 
In MRPT v3, these examples are designed as independent CMake projects that can be built standalone or within a `colcon` workspace.

---

## 🚀 How to Add a New Example

Follow these steps to add a new example to the MRPT tree:

1.  **Create the Directory**: 
    Create a folder named after your example in `[MRPT]/mrpt_examples_cpp/`.
    * *Example*: `[MRPT]/mrpt_examples_cpp/my_feature_demo/`.

2.  **Add Source Code**: 
    Save your C++ file (typically `main.cpp`) in that directory.

3.  **Register the Example**: 
    Open `generate_cmake_files.cmake` and add your example name to the relevant `LIST_EXAMPLES_IN_THIS_DIR` block.
    * Ensure you set `CMAKE_EXAMPLE_DEPS` using the v3 namespaced format (e.g., `mrpt::mrpt_math`).

4.  **Generate the Build Script**: 
    From the command line, run the generator script to create the local `CMakeLists.txt`:
    ```bash
    cmake -DMRPT_SOURCE_DIR=. -P generate_cmake_files.cmake
    ```

5.  **Commit Files**: 
    Add both your source code and the generated `CMakeLists.txt` to version control:
    ```bash
    git add my_feature_demo/main.cpp my_feature_demo/CMakeLists.txt
    ```

---

## 🛠 Build System Details

### Standalone Generation
The `generate_cmake_files.cmake` script uses `CMakeLists_template.txt.in` to produce a unique `CMakeLists.txt` for every example. Unlike previous versions, the v3 template "unrolls" dependencies so that each example contains explicit `find_package()` and `target_link_libraries()` calls without complex loops.

### Module Dependencies (v3)
Dependencies must be defined using the namespaced target format:
* **Correct**: `mrpt::mrpt_core`, `mrpt::mrpt_system`, `mrpt::mrpt_gui`.
* **Internal logic**: The generator automatically strips the `mrpt::` prefix to handle `find_package(mrpt_xxx)` while retaining the full namespace for linking.

---

## 📸 Optional: Documentation
To include your example in the official documentation:
1.  Add a `README.md` and a screenshot in the example directory.
2.  Place the screenshot in `[MRPT]/doc/source/images/`.
3.  Run `[MRPT]/scripts/generate_rst_docs_examples.sh`.