import numpy as np

#constants for use in the methods
MAX_DEPTH = 6 #start small (real AI use ~12-14)
#consider multithreading

#Function to run the move determination (wrapper for minmax)
#Function expects a Forsyth-Edwards Notation (please start on white side, wikipedia is backwards)
#function will return a character move
def get_chess_move(fen_board):
  #parse the input string
  bit_boards = np.empty(14,dtype=np.uint64) #create board array
  
  square = 0;
  ep_rank = ' '  #cache for en passant rank
  for c in fen_board:  #decipher each character, store location
    if(square < 64):  #still in chessboard definition
      if(c == 'P'): #ugh, no single line if statements
        bit_boards[0]  += 1 << square #white pawn
      elif(c == 'R'): 
        bit_boards[1]  += 1 << square #white rook
      elif(c == 'N'): 
        bit_boards[2]  += 1 << square #white knight
      elif(c == 'B'): 
        bit_boards[3]  += 1 << square #white bishop
      elif(c == 'Q'): 
        bit_boards[4]  += 1 << square #white queen
      elif(c == 'K'): 
        bit_boards[5]  += 1 << square #white king
      elif(c == 'p'): 
        bit_boards[6]  += 1 << square #black pawn
      elif(c == 'r'): 
        bit_boards[7]  += 1 << square #black rook
      elif(c == 'n'): 
        bit_boards[8]  += 1 << square #black knight
      elif(c == 'b'): 
        bit_boards[9]  += 1 << square #black bishop
      elif(c == 'q'): 
        bit_boards[10] += 1 << square #black queen
      elif(c == 'k'): 
        bit_boards[11] += 1 << square #black king
      elif(c >= '0' and c <= '9'):  #skip amount
        square += ord(c) - ord('0') - 1  #no char/int mapping either!?!
      if(c != '/'): #ignore row deliniations
        square += 1  #no ++? what is this, ancient rome?
    else: #in meta definitions now
      if(square == 65): #color on the move (assume white)
        if(c == 'b'): 
          bit_boards[13] += 1
      elif(square >= 67 and square <= 70): #castle avaliability
        if(c == 'K'): 
          bit_boards[13] += 2  #white kingside
        if(c == 'Q'): 
          bit_boards[13] += 4  #white queenside
        if(c == 'k'): 
          bit_boards[13] += 8  #black kingside
        if(c == 'q'): 
          bit_boards[13] += 16 #black queenside
        if(c == ' '): 
          square = 71  #finished early
      elif(square == 72): #en passant info
        if(c == '-'):
          bit_boards[12] = 0  #no en passant in effect
          square = 73          #skip ahead
        else: 
          ep_rank = (ord(c) - ord('a')) * 8    #cache rank
      elif(square == 73): 
        bit_boards[12] = ep_rank + ord(c) - '0'  #store ep
      #Not doing the 50-move stalemate counters, fuck it
      square += 1
      
  #now that that's done, start minmax (yay for pass-by-value)
  #best_move = alpha_beta_tree(bit_boards, MAX_DEPTH, None, None, False)
  #return best_move.tag
  return bit_boards


#class for storing moves internal to the minmax
class Move:
  def __init__(self, tag, value):
    self.tag = tag
    self.value = value

#recursive function to generate tree
#argumengs are: array of board bitmaps, remaining depth,
#  most recent min, most recent max, current operation (max/min)
#function returns the optimal move
def alpha_beta_tree(bit_boards, depth, last_min, last_max, is_min):
  return None



#function uses array of board bitmaps to generate a position's value
def get_state_evaluation(bit_boards):
  return None


#bit_board ordering is as follows:
#0  - white pawns
#1  - white rooks
#2  - white knights
#3  - white bishops
#4  - white queens
#5  - white kings
#6  - black pawns
#7  - black rooks
#8  - black knights
#9  - black bishops
#10 - black queens
#11 - black kings
#12 - en-passant square
#13 - meta info: bits[0=current move(white is 0),
#              1-4=castle avaliability (1/2 white, 1/3 king side, 2/4 queen side)]
#              Fuck stalemate counters, not doing it




