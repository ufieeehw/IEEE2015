#File defines all of the classes for use in the chess_ai file

import numpy as np

#storage class for the bit boards
class Board_State:
  #declare each piece board as a 64-bit unsigned integer (assume empty board)
  wp = np.uint64(0)
  wr = np.uint64(0)
  wn = np.uint64(0)
  wb = np.uint64(0)
  wq = np.uint64(0)
  wk = np.uint64(0)
  bp = np.uint64(0)
  br = np.uint64(0)
  bn = np.uint64(0)
  bb = np.uint64(0)
  bq = np.uint64(0)
  bk = np.uint64(0)
  ep = np.uint64(0) #en passant rule
  turn = True       #is it our turn?
  ai_color = True   #what color are we? (true = white, false = black)
  castle = 0        #castle avaliability-bits[(0/1 white, 0/2 king side, 1/3 queen side)]
  #not doing stalemate counters

  #construct the state from a FEN string and color
  def __init__(self, fen_board, color):
    square = 0;
    ep_file = ' '  #cache for en passant rank
    self.ai_color = color #store the ai color
    for c in fen_board:  #decipher each character, store location
      if(square < 64):  #still in chessboard definition
        if(c == 'P'): #ugh, no single line if statements
          self.wp += np.uint64(1<<square) #white pawn
        elif(c == 'R'): 
          self.wr += np.uint64(1<<square) #white rook
        elif(c == 'N'): 
          self.wn += np.uint64(1<<square) #white knight
        elif(c == 'B'): 
          self.wb += np.uint64(1<<square) #white bishop
        elif(c == 'Q'): 
          self.wq += np.uint64(1<<square) #white queen
        elif(c == 'K'): 
          self.wk += np.uint64(1<<square) #white king
        elif(c == 'p'): 
          self.bp += np.uint64(1<<square) #black pawn
        elif(c == 'r'): 
          self.br += np.uint64(1<<square) #black rook
        elif(c == 'n'): 
          self.bn += np.uint64(1<<square) #black knight
        elif(c == 'b'): 
          self.bb += np.uint64(1<<square) #black bishop
        elif(c == 'q'): 
          self.bq += np.uint64(1<<square) #black queen
        elif(c == 'k'): 
          self.bk += np.uint64(1<<square) #black king
        elif(c >= '0' and c <= '9'):  #skip amount
          square += ord(c) - ord('0') - 1  #no char/int mapping either!?!
        if(c != '/'): #ignore row deliniations
          square += 1  #no ++? what is this, ancient rome?
      else: #in meta definitions now
        if(square == 65): #color on the move
          if((c == 'w') == color): #(white's turn) !xor (ai is white)
            self.turn = True
          else:
            self.turn = False
        elif(square >= 67 and square <= 70): #castle avaliability
          if(c == 'K'): 
            self.castle += 1  #white kingside
          if(c == 'Q'): 
            self.castle += 2  #white queenside
          if(c == 'k'): 
            self.castle += 4  #black kingside
          if(c == 'q'): 
            self.castle += 8  #black queenside
          if(c == ' '): 
            square = 71  #finished early
        elif(square == 72): #en passant info
          if(c == '-'):
            self.ep = 0  #no en passant in effect
            square = 73   #skip ahead
          else: 
            ep_file = (ord(c) - ord('a'))  #cache rank
        elif(square == 73): 
          self.ep = 1 << ep_file + (ord(c) - ord('0')) << 8  #store ep
        #Not doing the 50-move stalemate counters, fuck it
        square += 1

  #return the locations of all white peices (int bitmaps are additive)
  def get_white_pieces(self):
    return self.wp+self.wr+self.wn+self.wb+self.wq+self.wk

  #return the locations of all black peices
  def get_black_pieces(self):
    return self.bp+self.br+self.bn+self.bb+self.bq+self.bk
  
  #return the locations of all peices  
  def get_all_pieces(self):
    return self.get_black_pieces() + self.get_white_pieces()

#class for storing moves internal to the minmax
class Move:
  def __init__(self, tag, value):
    self.tag = tag  #string in long-algebraic-notation
    self.value = value  #point value of the move
   
#function to compute the rank/file of a specific peice on a board    
def get_rank_file(board):
  r = 1
  f = 1
  temp = np.uint64(board) #cast input
  while (temp > 255):
    r += 1
    temp = np.uint64(temp/256) #>>8,rightshift doesn't like uint64
  while (temp > 1):
    f += 1
    temp = np.uint64(temp/2)   #>>1
  return (r,f)

