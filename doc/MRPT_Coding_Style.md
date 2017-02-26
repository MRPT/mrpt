 **C++ coding style for MRPT**
 -------------------------------
 
* **General rules**: Take a look at [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) for very good general advices and reasons to worry about code style. It is worth reading carefully.
* C++11/14 is **not** allowed unless working on a MRPT 2.0.0 branch.

**Class and file names**
* For historical reasons, most MRPT classes and structures use the prefix `C` and `T`, respectively. For example: `class CBar`and `struct TFoo`. This convention *should* be observed in new code for the sake of consistency. 
* A class named `CFoo` should have its declaration in a file named `CFoo.h` and its implementation in `CFoo.cpp`. Pure template classes may have detailed implementations in `CFoo_impl.h`.
* Each MRPT library, or "module", has its own directory under `MRPT/libs`, with its own `include` and `src` directory. More on source tree layout [here](http://www.mrpt.org/libs_tree_layout).
* Use the #include guard [`#pragma once`](https://en.wikipedia.org/wiki/Pragma_once) in new code.

**Code content and style**

* **Prefer tabs to spaces**. Yes, I know this is an eternal source of discussions and debate, but virtually all MRPT code follow this convention, so **please** respect it to avoid mixing whitespaces:

![spaces_tabs](https://raw.githubusercontent.com/MRPT/mrpt/master/doc/coding_style_mixed_space_tabs.png)

Enable **viewing whitespaces** in your editor to prevent errors (`CTRL+SHIFT+8` in Visual Studio) and change the default editor settings accordingly (QtCreator: Settings -> Editor -> ...).

* **Never**, **never**, put a `using namespace XXX;` in a header file, since it will pollute without control user namespaces. An exception is its use **inside** the scope of a function implementation in a header, e.g. an inline function or method.

* Member variables of a `struct` should have **no** suffix or prefix, e.g: 

        struct TFoo {
          int num_iters;
        };

* Public variable of a `class` should have **no** suffic or prefix. Private/protected members should have the `m_` prefix or, alternatively, the `_` suffix. For *methods*, use **lower camel case** or K&R style, e.g.

        class CBar {
        public:
          int num_iters;
          void doSomething(); // Lower camel case (preferred)
          void do_something(); // K&R style (second style option)
        private:
          int m_handle; 
        };

* In general, typedefs will use lowercase with underscores, e.g. `typedef std::vector<int> vector_int;`
 
* If a packed structure is defined (i.e. `#pragma pack(push,1) ... #pragma pack(pop)`), it will be much safer to make all fields protected and offer accessor methods. In this way, we avoid alignment errors in some processor architectures.
