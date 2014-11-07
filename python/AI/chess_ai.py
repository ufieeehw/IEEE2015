import numpy as np
import chess_ai_defs as ai

#constants for use in the methods
MAX_DEPTH = 6 #start small (real AI use ~12-14)

#considering multithreading


#Function to run the move determination (wrapper for minmax)
#Function expects a Forsyth-Edwards Notation (please start on white side, wikipedia is backwards)
#function will return a character move
def get_chess_move(fen_board):  
  state = ai.Board_State(fen_board) #construct the board state
      
  #now that that's done, start minmax (yay for pass-by-value)
  #best_move = alpha_beta_tree(bit_boards, MAX_DEPTH, None, None, False)
  #return best_move.tag
  return state #DEBUG

#recursive function to generate tree
#argumengs are: board state, remaining depth,
#  most recent min, most recent max, current operation (max/min)
#function returns the optimal move
def alpha_beta_tree(state, depth, last_min, last_max, is_min):
  return None

#function uses array of board bitmaps to generate a position's value
def get_state_evaluation(state):
  return None

#function returns a list of possible moves for a given state
def get_possible_moves(state):
  return None
