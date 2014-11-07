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
  white_turn = True #default to white's turn
  castle = 0        #castle avaliability-bits[(0/1 white, 0/2 king side, 1/3 queen side)]
  #not doing stalemate counters

  #construct the state from a FEN string
  def __init__(self, fen_board):
    square = 0;
    ep_rank = ' '  #cache for en passant rank
    for c in fen_board:  #decipher each character, store location
      if(square < 64):  #still in chessboard definition
        if(c == 'P'): #ugh, no single line if statements
          self.wp  += 1 << square #white pawn
        elif(c == 'R'): 
          self.wr  += 1 << square #white rook
        elif(c == 'N'): 
          self.wn  += 1 << square #white knight
        elif(c == 'B'): 
          self.wb  += 1 << square #white bishop
        elif(c == 'Q'): 
          self.wq  += 1 << square #white queen
        elif(c == 'K'): 
          self.wk  += 1 << square #white king
        elif(c == 'p'): 
          self.bp  += 1 << square #black pawn
        elif(c == 'r'): 
          self.br  += 1 << square #black rook
        elif(c == 'n'): 
          self.bn  += 1 << square #black knight
        elif(c == 'b'): 
          self.bb  += 1 << square #black bishop
        elif(c == 'q'): 
          self.bq += 1 << square #black queen
        elif(c == 'k'): 
          self.bk += 1 << square #black king
        elif(c >= '0' and c <= '9'):  #skip amount
          square += ord(c) - ord('0') - 1  #no char/int mapping either!?!
        if(c != '/'): #ignore row deliniations
          square += 1  #no ++? what is this, ancient rome?
      else: #in meta definitions now
        if(square == 65): #color on the move
          if(c == 'b'): 
            self.white_turn = False
          else:
            self.white_turn = True
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
            ep_rank = (ord(c) - ord('a')) * 8    #cache rank
        elif(square == 73): 
          self.ep = 1 << ep_rank + ord(c) - ord('0')  #store ep
        #Not doing the 50-move stalemate counters, fuck it
        square += 1

  #return the locations of all white peices (int bitmaps are additive)
  def get_white_peices(self):
    return self.wp+self.wr+self.wn+self.wb+self.wq+self.wk

  #return the locations of all black peices
  def get_black_peices(self):
    return self.bp+self.br+self.bn+self.bb+self.bq+self.bk
  
  #return the locations of all peices  
  def get_all_peices(self):
    return self.get_black_peices() + self.get_white_peices()

#class for storing moves internal to the minmax
class Move:
  def __init__(self, tag, value):
    self.tag = tag
    self.value = value
