# Contributing to OverclockedSwerve

:+1::tada: First off, thanks for taking the time to contribute!
:tada::+1: 

The purpose of this guide is to provide an overview of Overclocked's
GitHub workflow. These are just guidelines, not rules, use your best
judgment and feel free to propose changes to this document in a pull
request. 

If you are about to file a Pull Request, head down to the
[Pull Requests section](#pull-requests)

#### Table Of Contents

* [Issues](#issues)
  * [When working on an issue](#when-working-on-an-issue)

* [Committing](#committing)

* [Pull Requests](#pull-requests)
  * [Make sure your code is ready](#make-sure-your-code-is-ready)
  * [Filing the Pull Request](#filing-the-pull-request)

* [Writing JUnit tests](#writing-junit-tests)

## Issues 
Issues are created to keep track of new features and development
goals, in addition to bug-tracking. When filing a new issue, use your
best judgment when adding labels to issues, such as "bug",
"enhancement", "help wanted", and "question". Anybody is welcome to
work on any currently unassigned issue. If you want to work on an
issue, please write a comment in the thread of your intention and go
ahead and assign it to yourself.

### When working on an issue:
  * **Members of Overclocked**: please make a branch with
    a descriptive name based on the title of the issue. Names of
    branches should be all lowercase with words separated by
    dashes. Commit your work to that branch, and when ready, file a
    [pull request](#pull-requests).
  * **Non-Overclocked member**: first fork the project
    and commit to a branch with a descriptive name, following the same
    guidelines as above. When ready, file a [pull
    request](#pull-requests).

## Committing
You should be committing periodically while working in a branch. The
purpose of committing is to keep track of your changes, and have a way
to revert back if needed. Writing good commit messages is key to
having other team members understand your work. Please write the one
line commit "summary" in the present tense imperative with only the
first letter capitalized, e.g. "Add abz" or "Fix bla" instead of
"Added" or "Fixed". Use the optional extended description if needed,
writing there in whatever tense you find easiest.

Each time you push your code, your code gets compiled and tested by
[Travis](https://travis-ci.org/246overclocked/Overclocked-Swerve). Although you
should be [testing on your local machine as you develop](#make-sure-your-code-is-ready),
this is another good way to make sure you don't break anything.

## Pull Requests
When you think you are done working in a branch, file a pull
request. But, before filing a pull request, please make sure that your
code compiles and all tests pass. If you have code that is
testable, make sure you have written associated [JUnit tests](#writing-junit-tests) 
in the `test` directory.

### Make sure your code is ready
To ensure your code has not broken anything, run `ant test` in the
main directory of the project. If you see `BUILD SUCCESSFUL` then you
are ready to make a pull request. If you see `BUILD FAILED` you should
investigate the issue and fix your code before filing the pull
request. You can also check
[Travis](https://travis-ci.com/246overclocked/OverclockedSwerve) to see if
your branch is passing.

### Filing the Pull Request
Make a pull request on GitHub and assign it to one of the Overclocked
programming leaders. Make the title of the pull request a descriptive
summary of what the pull request does (features added, bugs fixed,
etc.). Use the body of the pull request to describe your pull request
in more detail, including any questions or comments you have for the
reviewer. 

**If your pull request closes one of the issues, the last
line of your description should be `Closes #x` where x = the issue
number.**

## Writing JUnit tests
The source code for the project all lives in packages under the `src`
directory. Associated tests should live in the `test` directory in
packages under the same name. Here is a simplified example:

```
├── src
    ├── org.usfirst.frc.team246.robot.commands
        └── CrabWithTwist.java
    ├── org.usfirst.frc.team246.robot.overclockedLibraries
        └── Vector2D.java
    └── org.usfirst.frc.team246.robot.subsystems
        └── Drivetrain.java
└── test
    ├── org.usfirst.frc.team246.robot.commands
        └──  CrabWithTwistTest.java
    └── org.usfirst.frc.team246.robot.overclockedLibraries
        └── Vector2DTest.java
    └── org.usfirst.frc.team246.robot.subsystems
        └── DrivetrainTest.java

```
Eclipse can help you set up JUnit tests automatically. See [this guide](https://courses.cs.washington.edu/courses/cse143/11wi/eclipse-tutorial/junit.shtml)
for more information.
