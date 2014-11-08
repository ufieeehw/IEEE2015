import numpy as np
import chess_ai_defs as ai

#globals/constants for use in the methods
MAX_DEPTH = 6 #start small (real AI use ~12-14)

#Function to run the move determination (wrapper for minmax)
#Function expects a Forsyth-Edwards Notation (please start on white side, wikipedia is backwards)
#  also expects the ai color (True=White, False=Black)
#function will return a character move in long-algebreic-notation
def get_chess_move(fen_board, color):  
  state = ai.Board_State(fen_board, color) #construct the board state
      
  #now that that's done, start minmax
  best_move = alpha_beta_tree(state, MAX_DEPTH, None, None, True)
  #TODO: Call Check (run evalutaion on free second move, see if king is dead)
  #TODO: Call Mate  (run evalutaion to a depth of 2, see if king survives)
  return best_move.tag

#recursive function to generate tree
#argumengs are: board state, remaining depth,
#  most recent min, most recent max, current operation (max/min)
#function returns the optimal move
def alpha_beta_tree(state, depth, last_min, last_max, is_max):
  move_strings = get_possible_moves(state) #get the strings corresponding to possible moves
  
  #TODO: map moves to cloned board states (create method in Board_State)
  #TODO: Call series of recursive methods to see if move can be beaten
  #TODO: Alpha/Beta pruning on Minmax tree
  #TODO: Multithread?
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
  moves = []  #empty list of move strings
  
  #map to static lists so both sides can run the same function
  if(state.turn == state.ai_color): #(ai color == white) !xor (turn == ai), white's turn
    opponent_pieces = state.get_black_pieces()
    friendly_pieces = state.get_white_pieces()
    piece_list = [state.wp, state.wr, state.wn, state.wb, state.wq, state.wk]
    pawn_dir = False   #pawns are moving forward
  else: #black's turn
    opponent_pieces = state.get_white_pieces()
    friendly_pieces = state.get_black_pieces()
    piece_list = [state.bp, state.br, state.bn, state.bb, state.bq, state.bk]
    pawn_dir = True  #pawns move backwards
  
  #generate possible moves. Priority is driven by ordering, so start with low-risk moves first
  #check if castles are avaliable
  if(state.ai_color and (state.castle & 1)):       #kingside castle
    if(not ((friendly_pieces + opponent_pieces) & np.uint64(0x00000000000000A0))): #spaces f1,g1 must be clear
      moves.append("O-O")
  if(state.ai_color and (state.castle & 2)):       #queenside castle
    if(not ((friendly_pieces + opponent_pieces) & np.uint64(0x000000000000000E))): #spaces b1,c1,d1 must be clear
      moves.append("O-O-O")
  if(not state.ai_color and (state.castle & 4)):   #kingside castle
    if(not ((friendly_pieces + opponent_pieces) & np.uint64(0xB000000000000000))): #spaces e8,f8,g8 must be clear
      moves.append("O-O")
  if(not state.ai_color and (state.castle & 8)):   #queenside castle
    if(not ((friendly_pieces + opponent_pieces) & np.uint64(0x0700000000000000))): #spaces b8,c8 must be clear
      moves.append("O-O-O")

  if(piece_list[0]): #check if there are pawns
    rank_file = ai.get_rank_file(piece_list[0])
    for piece in rank_file: #search for each piece found
      pawn_square = ai.get_square(piece)
      for r in (1, 2): #cycle through all avaliable angles of motion
        if(not pawn_dir):
          r = -1 * r  #invert pawn direction if we're black side
        for f in (-1, 0, 1):
          if(f != 0 and r == 2):
            continue #can't move forward twice when changing rank
          elif(f == 0 and (r == 2 or r == -2) and ((pawn_dir and piece[0] != 2) or (not pawn_dir and piece[0] != 7))):
            continue #pawn can only move two spaces from starting row
          if(piece[0]+r>0 and piece[0]+r<=8 and piece[1]+f>0 and piece[1]+f<=8): #check board boundaries
            new_square = pawn_square
            if(r > 0):  #shift as appropriate
              new_square = np.uint64(new_square * (1 << 8*r))  #numpy won't << with uint64
            elif(r < 0):
              new_square = np.uint64(new_square / (1 << 8*-r)) #numpy won't >> with uint64
            if(f > 0):
              new_square = np.uint64(new_square * (1 << f))    #<< by f*i
            elif(f < 0):
              new_square = np.uint64(new_square  / (1 << -f))  #>> by -f*i
            if(not ((new_square & friendly_pieces) or (f == 0 and (new_square & (friendly_pieces + opponent_pieces))) or (f != 0 and not (new_square & opponent_pieces)))): #no piece conflict
              move_string = chr(ord('a')+piece[1]-1) + chr(ord('0')+piece[0]) #old_location
              if(new_square & np.uint64(opponent_pieces + state.ep)): #piece capture
                move_string += 'x'
              else:
                move_string += '-'
              move_string += chr(ord('a')+piece[1]+f-1) + chr(ord('0')+piece[0]+r) #new location
              moves.append(move_string) #add it to the list

  #get the knights's moves, search +-1/2 in rank, and map file to 1/2 as appropriate
  if(piece_list[2]): #check if there is a knight
    rank_file = ai.get_rank_file(piece_list[2])
    for piece in rank_file: #search for each piece found
      knight_square = ai.get_square(piece)
      for r in (-2, -1, 1, 2): #cycle through all avaliable angles of motion
        for f in (-1, 1):
          if(r == -1 or r == 1):  #knight needs to 2-1 or 1-2, force this here
            f = f * 2
          if(piece[0]+(r)>0 and piece[0]+(r)<=8 and piece[1]+(f)>0 and piece[1]+(f) <=8): #check board boundaries
            new_square = knight_square
            if(r > 0):  #shift as appropriate
              new_square = np.uint64(new_square * (1 << 8*(r)))  #numpy won't << with uint64
            elif(r < 0):
              new_square = np.uint64(new_square / (1 << 8*(-r))) #numpy won't >> with uint64
            if(f > 0):
              new_square = np.uint64(new_square * (1 << (f)))    #<< by f*i
            elif(f < 0):
              new_square = np.uint64(new_square  / (1 << (-f)))  #>> by -f*i
            if(not (new_square & friendly_pieces)): #no piece conflict
              move_string = 'N' + chr(ord('a')+piece[1]-1) + chr(ord('0')+piece[0]) #old_location
              if(new_square & opponent_pieces): #piece capture
                move_string += 'x'
              else:
                move_string += '-'
              move_string += chr(ord('a')+piece[1]+(f)-1) + chr(ord('0')+piece[0]+(r)) #new location
              moves.append(move_string) #add it to the list
              if(new_square & opponent_pieces): #opponent piece
                break #opposing peice on board
            else:
              break #friendly piece blocking us
          else:
            break #edge of board, nowhere else to go
 
  #get the rook's moves, search each direction till obstructed
  if(piece_list[1]): #check if there is a rook
    rank_file = ai.get_rank_file(piece_list[1])
    for piece in rank_file: #search for each piece found
      rook_square = ai.get_square(piece)
      for r in range(-1, 1): #cycle through all avaliable angles of motion
        for f in (-1, 1):
          if(r != 0): #if we're moving vertically, don't move horizontally
            f = 0
          for i in range (1,8): #can move up to 7 squares
            if(piece[0]+(r*i)>0 and piece[0]+(r*i)<=8 and piece[1]+(f*i)>0 and piece[1]+(f*i) <=8): #check board boundaries
              new_square = rook_square
              if(r > 0):  #shift as appropriate
                new_square = np.uint64(new_square * (1 << 8*(r*i)))  #numpy won't << with uint64
              elif(r < 0):
                new_square = np.uint64(new_square / (1 << 8*(-r*i))) #numpy won't >> with uint64
              if(f > 0):
                new_square = np.uint64(new_square * (1 << (f*i)))    #<< by f*i
              elif(f < 0):
                new_square = np.uint64(new_square  / (1 << (-f*i)))  #>> by -f*i
              if(not (new_square & friendly_pieces)): #no piece conflict
                move_string = 'R' + chr(ord('a')+piece[1]-1) + chr(ord('0')+piece[0]) #old_location
                if(new_square & opponent_pieces): #piece capture
                  move_string += 'x'
                else:
                  move_string += '-'
                move_string += chr(ord('a')+piece[1]+(f*i)-1) + chr(ord('0')+piece[0]+(r*i)) #new location
                moves.append(move_string) #add it to the list
                if(new_square & opponent_pieces): #opponent piece
                  break #opposing peice on board
              else:
                break #friendly piece blocking us
            else:
              break #edge of board, nowhere else to go
          if(f==0):
            break #only need to check this condition once
   
  #get the bishop's moves, search each angle till obstructed
  if(piece_list[3]): #check if there is a bishop
    rank_file = ai.get_rank_file(piece_list[3])
    for piece in rank_file: #search for each piece found
      bishop_square = ai.get_square(piece)
      for r in (-1, 1): #cycle through all avaliable angles of motion
        for f in (-1, 1):
          for i in range (1,8): #can move up to 7 squares
            if(piece[0]+(r*i)>0 and piece[0]+(r*i)<=8 and piece[1]+(f*i)>0 and piece[1]+(f*i) <=8): #check board boundaries
              new_square = bishop_square
              if(r > 0):  #shift as appropriate
                new_square = np.uint64(new_square * (1 << 8*(r*i)))  #numpy won't << with uint64
              elif(r < 0):
                new_square = np.uint64(new_square / (1 << 8*(-r*i))) #numpy won't >> with uint64
              if(f > 0):
                new_square = np.uint64(new_square * (1 << (f*i)))    #<< by f*i
              elif(f < 0):
                new_square = np.uint64(new_square  / (1 << (-f*i)))  #>> by -f*i
              if(not (new_square & friendly_pieces)): #no piece conflict
                move_string = 'B' + chr(ord('a')+piece[1]-1) + chr(ord('0')+piece[0]) #old_location
                if(new_square & opponent_pieces): #piece capture
                  move_string += 'x'
                else:
                  move_string += '-'
                move_string += chr(ord('a')+piece[1]+(f*i)-1) + chr(ord('0')+piece[0]+(r*i)) #new location
                moves.append(move_string) #add it to the list
                if(new_square & opponent_pieces): #opponent piece
                  break #opposing peice on board
              else:
                break #friendly piece blocking us
            else:
              break #edge of board, nowhere else to go
     
   
  #get the queen's moves, search in each direction/angle till blocked
  if(piece_list[4]): #check if queen still exists
    rank_file = ai.get_rank_file(piece_list[4])
    for piece in rank_file: #search for each piece found (can have multiple queens)
      queen_square = ai.get_square(piece)  #get the queen's location
      for r in range(-1,2): #cycle through all avaliable angles of motion
        for f in range(-1,2):
          for i in range (1,8): #queen can move up to 7 squares
            if(piece[0]+(r*i)>0 and piece[0]+(r*i)<=8 and piece[1]+(f*i)>0 and piece[1]+(f*i) <=8): #check board boundaries
              new_square = queen_square
              if(r > 0):  #shift as appropriate
                new_square = np.uint64(new_square * (1 << 8*(r*i)))  #numpy won't << with uint64
              elif(r < 0):
                new_square = np.uint64(new_square / (1 << 8*(-r*i))) #numpy won't >> with uint64
              if(f > 0):
                new_square = np.uint64(new_square * (1 << (f*i)))    #<< by f*i
              elif(f < 0):
                new_square = np.uint64(new_square  / (1 << (-f*i)))  #>> by -f*i
              if(not (new_square & friendly_pieces)): #no piece conflict
                move_string = 'Q' + chr(ord('a')+piece[1]-1) + chr(ord('0')+piece[0]) #old_location
                if(new_square & opponent_pieces): #piece capture
                  move_string += 'x'
                else:
                  move_string += '-'
                move_string += chr(ord('a')+piece[1]+(f*i)-1) + chr(ord('0')+piece[0]+(r*i)) #new location
                moves.append(move_string) #add it to the list
                if(new_square & opponent_pieces): #opponent piece
                  break #opposing piece
              else:
                break #friendly piece blocking us
            else:
              break #edge of board, nowhere else to go
              
  #add the king's moves (ignore checks, state eval will catch them eventually)
  if(piece_list[5]): #check if king is still in play (suprisingly possible)
    king_square = piece_list[5] #get the king's square
    rank_file = ai.get_rank_file(king_square) #get the coordinates of the location
    piece = rank_file[0] #only one king ever
    for r in range(-1,2): #king can move 1 space in any direction (+1,0,-1)
      for f in range(-1,2):
        if(piece[0]+r>0 and piece[0]+r<=8 and piece[1]+f>0 and piece[1]+f <=8): #check board boundaries
          new_square = king_square  #storage for new location
          if(r > 0):  #shift as appropriate
            new_square = np.uint64(new_square * (1 << 8*(r*i)))  #numpy won't << with uint64
          elif(r < 0):
            new_square = np.uint64(new_square / (1 << 8*(-r*i))) #numpy won't >> with uint64
          if(f > 0):
            new_square = np.uint64(new_square * (1 << f))    #<< by f
          elif(f < 0):
            new_square = np.uint64(new_square  / (1 << -f))  #>> by -f
          if(not (new_square & friendly_pieces)): #no piece conflict
            move_string = 'K' + chr(ord('a')+piece[1]-1) + chr(ord('0')+piece[0]) #old_location
            if(new_square & opponent_pieces): #piece capture
              move_string += 'x'
            else:
              move_string += '-'
            move_string += chr(ord('a')+piece[1]+f-1) + chr(ord('0')+piece[0]+r) #new location
            moves.append(move_string) #add it to the list
            
  return moves
  
#Boo
