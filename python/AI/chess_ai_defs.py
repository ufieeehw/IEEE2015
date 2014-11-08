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
  def __init__(self, fen_board = None, color = True):
    if(fen_board == None): #empty constructor
      return
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

  #copy constructor
  def copy_board(self, other):
    self.wp = other.wp
    self.wr = other.wr
    self.wn = other.wn
    self.wb = other.wb
    self.wq = other.wq
    self.wk = other.wk
    self.bp = other.bp
    self.br = other.br
    self.bn = other.bn
    self.bb = other.bb
    self.bq = other.bq
    self.bk = other.bk    
    self.ep = other.ep
    self.turn = other.turn
    self.ai_color = other.ai_color
    self.castle = other.castle
  
  #Execute a  move
  def execute_move(self, move):
    if(move[0] >= 'A' and move[0] <= 'Z'):  #capitol letters are pieces
      if(move[0] == 'O'): #castle, manually execute swap
        if (len(move) == 3): #Kingside
          if(self.turn): #white
            self.execute_move("Ke1-g1")
            self.execute_move("Rh1-f1")
          else: #black
            self.execute_move("Ke8-g8")
            self.execute_move("Rh8-f8")
        else: #Queenside
          if(self.turn): #white
            self.execute_move("Ke1-c1")
            self.execute_move("Ra1-d1")
          else: #black
            self.execute_move("Ke8-c8")
            self.execute_move("Rh8-d8")
      piece_type = move[0]
      old_rank_file = (ord(move[2])-ord('0'),ord(move[1])-ord('a')+1) #starting location
      new_rank_file = (ord(move[5])-ord('0'),ord(move[4])-ord('a')+1) #new location
      capture = (move[3] == 'x') #was there a capture?
    else:
      piece_type = 'P'
      old_rank_file = (ord(move[1])-ord('0'),ord(move[0])-ord('a')+1) #starting location
      new_rank_file = (ord(move[4])-ord('0'),ord(move[3])-ord('a')+1) #new location
      capture = (move[2] == 'x') #was there a capture?

    old_square = get_square(old_rank_file) #get old square
    new_square = get_square(new_rank_file) #get new square
      
    if(self.turn):  #white's color
      if(piece_type == 'P'):
        self.wp = np.uint64((self.wp & ~old_square) | new_square) #remove old location, set new one
      if(piece_type == 'R'):
        self.wr = np.uint64((self.wr & ~old_square) | new_square) #remove old location, set new one
        if(old_rank_file[1] == 8): #kingside rook
          self.castle = self.castle & 0xE #remove castle availability
        else: #queenside rook 
          self.castle = self.castle & 0xD 
      if(piece_type == 'N'):
        self.wn = np.uint64((self.wn & ~old_square) | new_square) #remove old location, set new one
      if(piece_type == 'B'):
        self.wb = np.uint64((self.wb & ~old_square) | new_square) #remove old location, set new one
      if(piece_type == 'Q'):
        self.wq = np.uint64((self.wq & ~old_square) | new_square) #remove old location, set new one
      if(piece_type == 'K'):
        self.wk = np.uint64((self.wk & ~old_square) | new_square) #remove old location, set new one
        self.castle = (self.castle & 0xC) #remove castle avaliability
        
      if(capture):
        for target_type in ('P', 'R', 'N', 'B', 'Q', 'K'): #search for captured piece
          if(piece_type == 'P' and (self.bp & new_square)):
            self.bp = np.uint64(self.bp & ~new_square) #remove the piece
          if(piece_type == 'R' and (self.br & new_square)):
            self.bp = np.uint64(self.br & ~new_square) #remove the piece
          if(piece_type == 'N' and (self.bn & new_square)):
            self.bp = np.uint64(self.bn & ~new_square) #remove the piece
          if(piece_type == 'B' and (self.bb & new_square)):
            self.bp = np.uint64(self.bb & ~new_square) #remove the piece
          if(piece_type == 'Q' and (self.bq & new_square)):
            self.bp = np.uint64(self.bq & ~new_square) #remove the piece
          if(piece_type == 'K' and (self.bk & new_square)):
            self.bp = np.uint64(self.bk & ~new_square) #remove the piece
    
    else: #black's turn
      if(piece_type == 'P'):
        self.bp = np.uint64((self.bp & ~old_square) | new_square) #remove old location, set new one
      if(piece_type == 'R'):
        self.br = np.uint64((self.br & ~old_square) | new_square) #remove old location, set new one
        if(old_rank_file[1] == 8): #kingside rook
          self.castle = self.castle & 0xB #remove castle availability
        else: #queenside rook 
          self.castle = self.castle & 0x7 
      if(piece_type == 'N'):
        self.bn = np.uint64((self.bn & ~old_square) | new_square) #remove old location, set new one
      if(piece_type == 'B'):
        self.bb = np.uint64((self.bb & ~old_square) | new_square) #remove old location, set new one
      if(piece_type == 'Q'):
        self.bq = np.uint64((self.bq & ~old_square) | new_square) #remove old location, set new one
      if(piece_type == 'K'):
        self.bk = np.uint64((self.bk & ~old_square) | new_square) #remove old location, set new one
      if(capture):
        for target_type in ('P', 'R', 'N', 'B', 'Q', 'K'): #search for captured piece
          if(piece_type == 'P' and (self.wp & new_square)):
            self.wp = np.uint64(self.wp & ~new_square) #remove the piece
          if(piece_type == 'R' and (self.wr & new_square)):
            self.wp = np.uint64(self.wr & ~new_square) #remove the piece
          if(piece_type == 'N' and (self.wn & new_square)):
            self.wp = np.uint64(self.wn & ~new_square) #remove the piece
          if(piece_type == 'B' and (self.wb & new_square)):
            self.wp = np.uint64(self.wb & ~new_square) #remove the piece
          if(piece_type == 'Q' and (self.wq & new_square)):
            self.wp = np.uint64(self.wq & ~new_square) #remove the piece
          if(piece_type == 'K' and (self.wk & new_square)):
            self.wp = np.uint64(self.wk & ~new_square) #remove the piece
            self.castle = (self.castle & 0x3) #remove castle avaliability

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
#will return the rank/file of all matching items
def get_rank_file(board): 
  r = 1 #rank
  f = 1 #file
  board = np.uint64(board)
  mask = np.uint64(1)  #bitmask
  output = [] #output list
  while(r <= 8): #while we're still on the board
    if(board & mask): #check if bit is set
      output.append((r,f))
    f += 1  #increment file
    mask = np.uint64(mask * 2)  #shift mask
    if(f > 8):  #off of board
      f = 1     #reset file
      r += 1    #increment rank
  return output #return the list

def get_square(rank_file):
  val = np.uint64(1)
  for i in range(1,rank_file[0]):
    val = np.uint64(val*256)  #increment rank
  for i in range(1,rank_file[1]):
    val = np.uint64(val*2)  #increment file
  return val 
 
