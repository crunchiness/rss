You may want to use virtualenv when developing on your own machine.

Very quick instructions of how:
`virtualenv venv` to create `venv` folder which will hold all the libraries and stuff.
`source venv/bin/activate` to activate virtual environment. When activated your username in terminal should be prepended with "(venv)" 
`pip install -r requirements.txt` to install required libraries.
(`requirements.txt` contains libraries required by this project. Versions are the same as on DICE.)
`deactivate` to exit virtualenv.

Benefits are that you won't be accidentally using some random libraries from your machine, but only what we are supposed to use. Also you won't need to litter your system with libraries you will never use outside this project.