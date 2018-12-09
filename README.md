# ezkinematics

## Dependencies
Be sure you have PyGame installed, otherwise install it via `pip install pygame`

Also, install ezgame via `pip install ezgame`. This command should also install `point2d`.

These are all the dependencies needed. Please notice they use **Python3**.

## Usage
Edit `application.py` adding modes and loops accordingly. Each new mode should have an initialization added on the `Application.init()` method and a loop method created with in the format `<mode>_loop`.

To move the arm, use a controller class. See `incrementer.py` to get an idea of how to do it.
