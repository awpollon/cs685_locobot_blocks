python3 -ic 'from blockbot import BlockBot
import math
import sys
from math import pi
from gtsam import symbol_shorthand
L = symbol_shorthand.L
X = symbol_shorthand.X
file_path = "../logs/bot.log"
sys.stdout = open(file_path, "w")
bot = BlockBot()'
