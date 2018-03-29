# C++ coding style for MRPT

## General rules

* Take a look at [Google's C++ style
guide](https://google.github.io/styleguide/cppguide.html) for very good general
advices and reasons to worry about code style. It is worth reading carefully.

* C++11/14 is **not** allowed unless working on a MRPT 2.0.0 branch.

## Class and filenames

* For historical reasons, most MRPT classes and structures use the prefix `C`
    and `T`, respectively. For example: `class CBar`and `struct TFoo`. This
    convention *should* be observed in new code for the sake of consistency.
* A class named `CFoo` should have its declaration in a file named `CFoo.h` and
    its implementation in `CFoo.cpp`. Pure template classes may have detailed
    implementations in `CFoo_impl.h`.
* Each MRPT library, or "module", has its own directory under `MRPT/libs`, with
    its own `include` and `src` directory. More on source tree layout
    [here](http://www.mrpt.org/libs_tree_layout).
* Use the #include guard [`#pragma
    once`](https://en.wikipedia.org/wiki/Pragma_once) in new code.

## Code content and style

### Tabs vs Spaces

Prefer **tabs to spaces**. Yes, I know this is an eternal source of discussions
and debate, but virtually all MRPT code follow this convention, so **please**
respect it to avoid mixing whitespaces like in the following case:

More specifically:

* Enable *viewing of whitespaces* in your editor of choice, e.g.,:

    * Visual Studio: `ctrl+shift+8`
    * Vim:
        ```vim
        set listchars=eol:¬,tab:>·,trail:~,extends:>,precedes:<
        set list
        ```
* Change the default editor settings so that you
    * Use 1 tab for indentation
        * Use an additional tab for *hanging indentation* (e.g. for open parenthesis
        cases)
    * Use spaces for splitting words in the same line, aligning code snippets
        etc.

In the following example `>.` designates a tab character. If the latter is not
there, a space is assumed.

**Wrong indentation**

![mixed_indentation2](https://raw.githubusercontent.com/MRPT/mrpt/master/doc/design_of_images/vim_wrong_indentation.png)

**Correct indentation**

![mixed_indentation2](https://raw.githubusercontent.com/MRPT/mrpt/master/doc/design_of_images/vim_correct_indentation.png)

**Wrong inline alignment**

![wrong_inline_alignment](https://raw.githubusercontent.com/MRPT/mrpt/master/doc/design_of_images/wrong_inline_alignment.png)

**Correct inline alignment**

![correct_inline_alignment](https://raw.githubusercontent.com/MRPT/mrpt/master/doc/design_of_images/correct_inline_alignment.png)

Finally, if you are a vim-user you can add [this snippet of
code](https://gist.github.com/bergercookie/9a2e96e19733b32ca55b8e2940eaba2c) to
your `.vimrc` to enable the aforementioned settings.

### Misc

* **Never**, **ever**, put a `using namespace XXX;` in a header file, since it
    will pollute without control user namespaces. An exception is its use
    **inside** the scope of a function implementation in a header, e.g. an
    inline function or method.

* Member variables of a `struct` should have **no** suffix or prefix, e.g:

  ```c++
  struct TFoo {
    int num_iters;
  };
  ```

* Public variable of a `class` should have **no** suffic or prefix.
    Private/protected members should have the `m_` prefix or, alternatively, the
    `_` suffix. For *methods*, use **lower camel case** or K&R style, e.g.

  ```c++
  class CBar {
  public:
    int num_iters;
    void doSomething(); // Lower camel case (preferred)
    void do_something(); // K&R style (second style option)
  private:
    int m_handle;
  };
  ```

* In general, `typedefs` and `using` will use lowercase with underscores, e.g.
   `using vector_int = std::vector<int>;`

* If a packed structure is defined (i.e. `#pragma pack(push,1) ... #pragma
    pack(pop)`), it will be much safer to make all fields protected and offer
    accessor methods. In this way, we avoid alignment errors in some processor
    architectures.
