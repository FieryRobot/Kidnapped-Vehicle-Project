# Kidnapped Car Project
This repository contains my submission for the Localization project that's part of Udacity's Self-Driving Car Nanodegree, term 2.

The original repo for this project, with instructions, is [here](https://github.com/udacity/CarND-Kidnapped-Vehicle-Project).

You can use the scripts contained in here to help build (build.sh, run.sh, etc.).

If you are on a Mac, you can do: `cmake -G Xcode` and you'll get an Xcode project to use.

I'm using designated initializers in here, which your compiler may not like. This means my code like this might not compile for you:

```
        nearby_landmarks.push_back({
          .x = l.x_f,
          .y = l.y_f,
          .id = l.id_i
        });
```

So you'll need to modify it if the compiler complains about complex initializers.

I had considered writing some wrappers to help me do things like map/filter/reduce, but decided to just use loops.

I make no claim for any of the code here except for my additions to the particle_filter.cpp/.h files.