#File defines all of the classes for use in the chess_ai file

#storage class for the bit boards
class Board_State:
  #declare each piece board as a 64-bit unsigned integer (assume empty board)
  wp = 0
  wr = 0
  wn = 0
  wb = 0
  wq = 0
  wk = 0
  bp = 0
  br = 0
  bn = 0
  bb = 0
  bq = 0
  bk = 0
  ep = 0 #en passant rule
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
          self.wp += 1<<square #white pawn
        elif(c == 'R'): 
          self.wr += 1<<square #white rook
        elif(c == 'N'): 
          self.wn += 1<<square #white knight
        elif(c == 'B'): 
          self.wb += 1<<square #white bishop
        elif(c == 'Q'): 
          self.wq += 1<<square #white queen
        elif(c == 'K'): 
          self.wk += 1<<square #white king
        elif(c == 'p'): 
          self.bp += 1<<square #black pawn
        elif(c == 'r'): 
          self.br += 1<<square #black rook
        elif(c == 'n'): 
          self.bn += 1<<square #black knight
        elif(c == 'b'): 
          self.bb += 1<<square #black bishop
        elif(c == 'q'): 
          self.bq += 1<<square #black queen
        elif(c == 'k'): 
          self.bk += 1<<square #black king
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
  
  #Execute a  move (swap indicates that we want to take turns)
  def execute_move(self, move, swap=True):
    if(move[0] >= 'A' and move[0] <= 'Z'):  #capital letters are pieces
      if(move[0] == 'O'): #castle, manually execute swap
        if (len(move) == 3): #Kingside
          if(self.turn): #white
            self.execute_move("Ke1-g1",False)
            self.execute_move("Rh1-f1",False)
          else: #black
            self.execute_move("Ke8-g8",False)
            self.execute_move("Rh8-f8",False)
        else: #Queenside
          if(self.turn): #white
            self.execute_move("Ke1-c1",False)
            self.execute_move("Ra1-d1",False)
          else: #black
            self.execute_move("Ke8-c8",False)
            self.execute_move("Rh8-d8",False)
        return #break method
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
      
    if(self.turn and self.ai_color):  #white's turn
      if(piece_type == 'P'):
        self.wp = (self.wp & ~old_square) | new_square #remove old location, set new one
      if(piece_type == 'R'):
        self.wr = (self.wr & ~old_square) | new_square #remove old location, set new one
        if(old_rank_file[1] == 8): #kingside rook
          self.castle = self.castle & 0xE #remove castle availability
        else: #queenside rook 
          self.castle = self.castle & 0xD 
      if(piece_type == 'N'):
        self.wn = (self.wn & ~old_square) | new_square #remove old location, set new one
      if(piece_type == 'B'):
        self.wb = (self.wb & ~old_square) | new_square #remove old location, set new one
      if(piece_type == 'Q'):
        self.wq = (self.wq & ~old_square) | new_square #remove old location, set new one
      if(piece_type == 'K'):
        self.wk = (self.wk & ~old_square) | new_square #remove old location, set new one
        self.castle = (self.castle & 0xC) #remove castle avaliability
        
      if(capture):
        for target_type in ('P', 'R', 'N', 'B', 'Q', 'K'): #search for captured piece
          if(target_type == 'P' and (self.bp & new_square)):
            self.bp = self.bp & ~new_square #remove the piece
          if(target_type == 'R' and (self.br & new_square)):
            self.br = self.br & ~new_square #remove the piece
          if(target_type == 'N' and (self.bn & new_square)):
            self.bn = self.bn & ~new_square #remove the piece
          if(target_type == 'B' and (self.bb & new_square)):
            self.bb = self.bb & ~new_square #remove the piece
          if(target_type == 'Q' and (self.bq & new_square)):
            self.bq = self.bq & ~new_square #remove the piece
          if(target_type == 'K' and (self.bk & new_square)):
            self.bk = self.bk & ~new_square #remove the piece
    
    else: #black's turn
      if(piece_type == 'P'):
        self.bp = (self.bp & ~old_square) | new_square #remove old location, set new one
      if(piece_type == 'R'):
        self.br = (self.br & ~old_square) | new_square #remove old location, set new one
        if(old_rank_file[1] == 8): #kingside rook
          self.castle = self.castle & 0xB #remove castle availability
        else: #queenside rook 
          self.castle = self.castle & 0x7 
      if(piece_type == 'N'):
        self.bn = (self.bn & ~old_square) | new_square #remove old location, set new one
      if(piece_type == 'B'):
        self.bb = (self.bb & ~old_square) | new_square #remove old location, set new one
      if(piece_type == 'Q'):
        self.bq = (self.bq & ~old_square) | new_square #remove old location, set new one
      if(piece_type == 'K'):
        self.bk = (self.bk & ~old_square) | new_square #remove old location, set new one
      if(capture):
        for target_type in ('P', 'R', 'N', 'B', 'Q', 'K'): #search for captured piece
          if(target_type == 'P' and (self.wp & new_square)):
            self.wp = self.wp & ~new_square #remove the piece
          if(target_type == 'R' and (self.wr & new_square)):
            self.wr = self.wr & ~new_square #remove the piece
          if(target_type == 'N' and (self.wn & new_square)):
            self.wn = self.wn & ~new_square #remove the piece
          if(target_type == 'B' and (self.wb & new_square)):
            self.wb = self.wb & ~new_square #remove the piece
          if(target_type == 'Q' and (self.wq & new_square)):
            self.wq = self.wq & ~new_square #remove the piece
          if(target_type == 'K' and (self.wk & new_square)):
            self.wk = self.wk & ~new_square #remove the piece
            self.castle = (self.castle & 0x3) #remove castle avaliability
    if(swap): self.turn = not self.turn #next player's turn

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
  search_result = [] #output list
  for i in range(0,64): #search each bit
    if((1 << i) & board): search_result.append(i) #store found pieces
  
  #lambda to turn index into rank/file
  resolve = lambda x: ((x >> 3)+1,(x%8)+1)
  return map(resolve, search_result) #return the list

def get_square(rank_file):
  val = 1 << ((rank_file[0]-1) << 3)  #increment rank (rf[0]<<3 = rf[0]*8)
  val = val << (rank_file[1]-1)       #increment file 
  return val 

