
  ================
  K-MEANS++ README
  ================


This is an efficient implementation and test suite for k-means and k-means++. k-means++ is a
variant on the standard k-means (Lloyd's method) that is faster and more accurate in practice,
while also achieving a provable approximation guarantee, something that standard k-means does
not. For more information on k-means++, see:

  http://www.stanford.edu/~darthur/kMeansPlusPlus.pdf

and Chapter 6 of:

  http://www.stanford.edu/~darthur/thesis.pdf

The k-means steps are done using the Filtering algorithm of Kanungo and Mount, which speeds up
any k-means style algorithm by a great deal:

  http://www.cs.umd.edu/~mount/Papers/pami02.pdf

This is a beta version. If you find any bugs, please let me know: darthur@gmail.com


- David Arthur


  ===
  FAQ
  ===


Q: What is k-means?

  http://en.wikipedia.org/wiki/K-means_algorithm

Q: What is k-means++?

  It is a very specific way of choosing the initial centers for k-means.

Q: Why should I use k-means++ instead of k-means?

  It generates better clusterings than standard k-means on virtually all data sets. It runs faster
  than standard k-means on average. It has a theoretical approximation guarantee.

Q: What do you mean when you say k-means++ has a theoretical approximation guarantee?

  The k-means algorithm is attempting to choose a clustering that minimizes the cost function:

    sum distance(x, c(x))^2

  where the sum is over all x in the data set, and c(x) is the center closest to x. k-means++
  guarantees that the expected cost achieved on ANY data set is within a factor of O(log k) of the
  optimal cost for that data set. Standard k-means offers no such guarantees. It is easy to construct
  cases where standard k-means does arbitrarily badly with high probability.

Q: Can you explain how k-means++ works in more detail?

  k-means++ is just a way of choosing the initial centers for k-means. After that, we run k-means as
  normal. So, suppose we want to choose k initial centers from a point-set (x_1, x_2, ..., x_n). Here
  is the full algorithm to choose a set C of centers:

    1. Choose one point uniformly at random from (x_1, x_2, ..., x_n), and add it to C.
    2. For each point x_i, set D(x_i) to be the distance between x_i and the nearest point in C.
    3. Choose a real number y uniformly at random between 0 and D(x_1)^2 + D(x_2)^2 + ... + D(x_n)^2.
    4. Find the unique integer i so that
         D(x_1)^2 + D(x_2)^2 + ... D(x_i)^2 >= y > D(x_1)^2 + D(x_2)^2 + ... + D(x_(i-1))^2.
    5. Add x_i to C.
    6. Repeat Steps 2-5 until we have chosen k centers.

Q: Why bother with randomness? Wouldn't it just be better to choose x_i with maximum D(x_i) instead?

  Absolutely not. This is a bad idea for several reasons:
    1. It performs worse in practice.
    2. Unlike k-means++, this way offers no approximation guarantees.
    3. It virtually guarantees you will pick every outlier as a center. That's bad.
    4. It is not random. If a method is random, you can run it many times and take the best clustering.
       Randomness is a GOOD thing for k-means.

Q: How do I compile this? Do I need extra libraries?

  The only library used is STL, which every reasonable version of C++ should come with. I have tested
  this implementation on Visual Studio 2008 and on g++. Regardless of what compiler you are using, you
  should turn on optimizations (use release mode in Visual Studio and use the -O2 flag in g++).

Q: What is a good way to view the output after running TestKm.cpp?

  The columns in the table are tab-separated. I like to import it into MS Excel.

Q: This implementation is complicated and hard to read. Is there a simpler one?

  http://www.stanford.edu/~darthur/kMeansppTest.zip
  However, that implementation is much slower. It is easier to read, but you should not use it in
  practice.

Q: I want to use k-means++ in my own project. How do I do it?

  Add the files KMeans.cpp, KMeans.h, KmUtils.cpp, KmUtils.h, KmTree.cpp, KmTree.h to your project. Use
  the functions in KMeans.h. It should be self-explanatory. You may use and modify the code as you see
  fit, but please maintain a reference in the comments to this implementation:

  http://www.stanford.edu/~darthur/kmpp.zip
