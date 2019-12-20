# modular-slam

## clang-format before commit (pre-commit hook)
Before install hook be sure that you have **clang-format** installed. 
To install clang-format on ubuntu run:

`sudo apt-get install clang-format`

To install git pre-commit hooks in project root directory run:


    cp ci/scripts/pre-commit .git/hooks/
    chmod +x .git/hooks/pre-commit
