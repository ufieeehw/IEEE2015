import numpy as np
import chess_ai_defs as ai

#constants for use in the methods
MAX_DEPTH = 6 #start small (real AI use ~12-14)

#considering multithreading


#Function to run the move determination (wrapper for minmax)
#Function expects a Forsyth-Edwards Notation (please start on white side, wikipedia is backwards)
#  also expects the ai color (True=White, False=Black)
#function will return a character move
def get_chess_move(fen_board, color):  
  state = ai.Board_State(fen_board, color) #construct the board state
      
  #now that that's done, start minmax (yay for pass-by-value)
  #best_move = alpha_beta_tree(bit_boards, MAX_DEPTH, None, None, False)
  #return best_move.tag
  return get_state_evaluation(state) #DEBUG

#recursive function to generate tree
#argumengs are: board state, remaining depth,
#  most recent min, most recent max, current operation (max/min)
#function returns the optimal move
def alpha_beta_tree(state, depth, last_min, last_max, is_min):
  return None

#function counts the peices remaining on the board, and multiplies by thier weight (Kauffman's 2012 values)
#super simple first draft, will probably improve later
def get_state_evaluation(state):
  score = 0 #position is initially neutral
  score += bin(state.wp).count("1")*100 #count the 1's in the bitmap
  score -= bin(state.bp).count("1")*100 #subtract black pawns (value 100)
  score += bin(state.wr).count("1")*525 #white rooks (value 525)
  score -= bin(state.br).count("1")*525 #black rooks (value 525)
  score += bin(state.wn).count("1")*350 #white knights (350)
  score -= bin(state.bn).count("1")*350 #black knights (350)
  score += bin(state.wb).count("1")*350 #white bishop (350)
  score -= bin(state.bb).count("1")*350 #black bishop (350)
  if(state.wq):
    score += 1000 #white queen (1000)
  if(state.bq):
    score -= 1000 #black queen (1000)
  if(state.wk):
    score += 100000 #white king (100,000)  (take the king instead of checkmating)
  if(state.bk):
    score -= 100000 #black king (100,000)
  
  if(not state.ai_color): #adjust to player color
    score = -score
  return score #return the result

#function returns a list of possible moves for a given state
def get_possible_moves(state):
  return None
