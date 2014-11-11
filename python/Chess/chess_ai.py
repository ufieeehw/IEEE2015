import numpy as np
import chess_ai_defs as ai
import time
from random import randrange
from multiprocessing import Pool
from threading import Lock

#globals/constants for use in the methods
MAX_DEPTH = 4 #best I've got so far (target is ~8-10) (real AI use ~12-14)
MAX_THREADS = 2 #number of threads to run, optimal value depends on system (~cores)

#consider moving multithreading to task queue
#should teach it default move

#Function to run the move determination (wrapper for minmax)
#Function expects a Forsyth-Edwards Notation (please start on white side, wikipedia is backwards)
#  also expects the ai color (True=White, False=Black)
#function will return a character move in long-algebreic-notation
def get_chess_move(fen_board, color):  
  state = ai.Board_State(fen_board, color) #construct the board state, make it available globally
  global global_state   #make top state avaliable to threads
  global top_max        #make top max avaliable to threads
  global lock           #make a lock avaliable to the threads
  global_state = state  
  top_max = -900000     #need to assign, start at impossibly low value
  lock = Lock()
  
  #do some timing things
  start = time.time()
  
  #Start the search
  pool = Pool(MAX_THREADS) #spawn specified number of worker threads
  move_strings = get_possible_moves(state) #get all possible first moves
  moves = pool.map(do_search_thread, move_strings) #run the threads
  pool.close() #close threads after they finish
  pool.join() #wait for threads to terminate before continuing
  
  best_move = moves[0]
  all_equal = True  #check if all moves are equal
  for move in moves[1:]: #get best move
    if(move.value > best_move.value): 
      best_move = move
      all_equal = False
      print 'better_move'
  
  #pick random move instead if all moves are the same value
  if(all_equal):
    best_move = moves[randrange(len(moves))]
  
  #debug (single thread instead of multi)
  #best_move = alpha_beta_tree(state, MAX_DEPTH, None, None, True)

  #See if we've checked them (give ai a free second move, see if we capture the king)
  new_state = ai.Board_State() #create the new state
  new_state.copy_board(state)
  new_state.execute_move(best_move.tag,False) #execute our chosen move and take another turn
  check_check = alpha_beta_tree(new_state, 1, None, None, True) #check tree
  new_state.execute_move(check_check.tag) #execute the free move
  if(new_state.ai_color and not new_state.bk): print 'Check'
  elif(not new_state.ai_color and not new_state.wk): print 'Check'
    
  #See if we've checkmated them (iterate one round, see if they survive)
  new_state.copy_board(state)
  new_state.execute_move(best_move.tag) #execute our chosen move
  check_check = alpha_beta_tree(new_state, 2, None, None, False) #do thier turn
  new_state.execute_move(check_check.tag)
  check_check = alpha_beta_tree(new_state, 1, None, None, True)  #do our turn
  new_state.execute_move(check_check.tag)
  if(new_state.ai_color and not new_state.bk): print 'Checkmate'
  elif(not new_state.ai_color and not new_state.wk): print 'Checkmate'
  
  return best_move.tag

#wrapper function for tree search, will spawn single search thread
#function must run from top level, and always assumes the ai is moving
def do_search_thread(move):
  global global_state   #use global state
  global top_max        #use global max
  global lock           #use global lock
  
  #print move
  
  new_state = ai.Board_State()  #create a new state
  new_state.copy_board(global_state) #copy the main state
  new_state.execute_move(move)  #execute the move
  
  lock.acquire() #get the lock
  gmax = top_max #copy the global max
  lock.release() #release the lock
  
  best_move = alpha_beta_tree(new_state, MAX_DEPTH-1, gmax, None, False) #throw to tree search for next iteration

  lock.acquire() #get the lock
  if(top_max < best_move.value): #check if we're better
    top_max = best_move.value #store new best value
  lock.release() #release the lock
  
  best_move.tag = move
  return best_move

#recursive function to generate tree
#argumengs are: board state, remaining depth,
#  most recent min, most recent max, current operation (max/min)
#function returns the optimal move
def alpha_beta_tree(state, depth, last_max, last_min, is_max):
  if(depth == 0): #end of recursive function
    return ai.Move("", get_state_evaluation(state))  #return the value of this position
    
  move_strings = get_possible_moves(state) #get the strings corresponding to possible moves
  if(len(move_strings) == 0): return ai.Move("",get_state_evaluation(state)) #handle edge case
  
  new_state = ai.Board_State()
  new_state.copy_board(state)
  new_state.execute_move(move_strings[0])
  if(is_max): best_move = alpha_beta_tree(new_state, depth-1, None, last_min, False)
  else: best_move = alpha_beta_tree(new_state, depth-1, last_max, None, True)
  best_move.tag = move_strings[0]  
  
  for move in move_strings[1:]:   #search each other move
    new_state.copy_board(state)   #copy the board
    new_state.execute_move(move)  #execute the move
    if(is_max): contender = alpha_beta_tree(new_state, depth-1, best_move.value, last_min, False) #recurse!
    else: contender = alpha_beta_tree(new_state, depth-1, last_max, best_move.value, True) #recurse!
    if((is_max and contender.value >= best_move.value) or (not is_max and contender.value <= best_move.value)):
      best_move = contender #contender is better
      best_move.tag = move  #update the tag
      #Do alpha beta - remove any value which cannot possiby replace an existing one
      if(last_max != None and last_min != None):
        if(best_move.value >= last_min or best_move.value <= last_max): return best_move 
      elif(last_max != None and best_move.value <= last_max): return best_move
      elif(last_min != None and best_move.value >= last_min):  return best_move
  
  return best_move

#function counts the peices remaining on the board, and multiplies by thier weight (Kauffman's 2012 values)
#super simple first draft, will probably improve later
def get_state_evaluation(state):
  score = 0 #position is initially neutral
  score += bin(state.wp).count("1")*100  #count the 1's in the bitmap
  score -= bin(state.bp).count("1")*100  #subtract black pawns (value 100)
  score += bin(state.wr).count("1")*525  #white rooks (value 525)
  score -= bin(state.br).count("1")*525  #black rooks (value 525)
  score += bin(state.wn).count("1")*350  #white knights (350)
  score -= bin(state.bn).count("1")*350  #black knights (350)
  score += bin(state.wb).count("1")*350  #white bishop (350)
  score -= bin(state.bb).count("1")*350  #black bishop (350)
  score += bin(state.wb).count("1")*1000 #white queen (1000)
  score -= bin(state.bb).count("1")*1000 #black queen (1000)
  if(state.wk != 0): score += 100000 #white king (100,000)  (take the king instead of checkmating)
  if(state.bk != 0): score -= 100000 #black king (100,000)
  
  if(not state.ai_color): score = -score #adjust to player color
  
  #incentivize trades by subtracting total piece count
  score -= bin(state.get_all_pieces()).count('1') * 25
  
  return score #return the result

#function returns a list of possible moves for a given state
def get_possible_moves(state):
  gen_moves = []  #empty list of move strings
  append_move = gen_moves.append #store function reference (optimization)
  
  #map to static lists so both sides can run the same function
  if(state.turn == state.ai_color): #(ai color == white) !xor (turn == ai), white's turn
    opponent_pieces = state.get_black_pieces()
    friendly_pieces = state.get_white_pieces()
    piece_list = [state.wp, state.wr, state.wn, state.wb, state.wq, state.wk]
    pawn_dir = True   #pawns are moving forward
  else: #black's turn
    opponent_pieces = state.get_white_pieces()
    friendly_pieces = state.get_black_pieces()
    piece_list = [state.bp, state.br, state.bn, state.bb, state.bq, state.bk]
    pawn_dir = False  #pawns move backwards
  
  #generate possible moves. Try moves more likely to resort in favorable situations first to improve search time
  #check if castles are avaliable
  if(state.ai_color and (state.castle & 1)):       #kingside castle
    if(not ((friendly_pieces + opponent_pieces) & 0x0000000000000030)): #spaces f1,g1 must be clear
      append_move("O-O")
  if(state.ai_color and (state.castle & 2)):       #queenside castle
    if(not ((friendly_pieces + opponent_pieces) & 0x000000000000000F)): #spaces b1,c1,d1 must be clear
      append_move("O-O-O")
  if(not state.ai_color and (state.castle & 4)):   #kingside castle
    if(not ((friendly_pieces + opponent_pieces) & 0x3000000000000000)): #spaces f8,g8 must be clear
      append_move("O-O")
  if(not state.ai_color and (state.castle & 8)):   #queenside castle
    if(not ((friendly_pieces + opponent_pieces) & 0x0F00000000000000)): #spaces b8,c8,d8 must be clear
      append_move("O-O-O")

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
            #shift as appropriate
            if(r > 0): new_square = new_square << (r<<3)     #leftshift by 8 bits per row
            elif(r < 0): new_square = new_square >> ((-r)<<3)  #rightshift (r<<3 = r*8)
            if(f > 0): new_square = new_square << f     #<< by f
            elif(f < 0): new_square = new_square >> -f  #>> by -f
            #check for piece conflicts
            if(not ((new_square & friendly_pieces) or (f == 0 and (new_square & (friendly_pieces + opponent_pieces))) or (f != 0 and not (new_square & opponent_pieces)))):
              if(new_square & opponent_pieces + state.ep): capture = 'x' #piece captured
              else: capture = '-' #no capture
              #store the ouput string (old_location+capture+new_location)
              move_string = "%s%s%s%s%s" % (chr(ord('a')+piece[1]-1),chr(ord('0')+piece[0]),capture,chr(ord('a')+piece[1]+f-1),chr(ord('0')+piece[0]+r))
              append_move(move_string) #add it to the list

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
            #shift as appropriate
            if(r > 0): new_square = new_square << (r<<3)     #leftshift by 8 bits per row (r<<3 = r*8)
            elif(r < 0): new_square = new_square >> ((-r)<<3)  #rightshift 
            if(f > 0): new_square = new_square << f     #<< by f
            elif(f < 0): new_square = new_square >> -f  #>> by -f
            if(not (new_square & friendly_pieces)): #no piece conflict
              if(new_square & opponent_pieces + state.ep): capture = 'x' #piece captured
              else: capture = '-' #no capture
              #store the ouput string (N+old_location+capture+new_location)
              move_string = "N%s%s%s%s%s" % (chr(ord('a')+piece[1]-1),chr(ord('0')+piece[0]),capture,chr(ord('a')+piece[1]+f-1),chr(ord('0')+piece[0]+r))
              append_move(move_string) #add it to the list
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
      for r in (-1, 0, 1): #cycle through all avaliable angles of motion
        for f in (-1, 1):
          if(r != 0): #if we're moving vertically, don't move horizontally
            if(f == -1): f = 0 #check vertical move
            else: break #keep from checking a move multiple times
          for i in range (1,8): #can move up to 7 squares
            if(piece[0]+(r*i)>0 and piece[0]+(r*i)<=8 and piece[1]+(f*i)>0 and piece[1]+(f*i) <=8): #check board boundaries
              new_square = rook_square
              #shift as appropriate
              if(r > 0): new_square = new_square << (i<<3)     #leftshift by 8 bits per row
              elif(r < 0): new_square = new_square >> (i<<3)  #rightshift (i<<3 = i*8)
              if(f > 0): new_square = new_square << i     #<< by f
              elif(f < 0): new_square = new_square >> i   #>> by -f
              if(not (new_square & friendly_pieces)):   #no piece conflict
                if(new_square & opponent_pieces + state.ep): capture = 'x' #piece captured
                else: capture = '-' #no capture
                #store the ouput string (N+old_location+capture+new_location)
                move_string = "R%s%s%s%s%s" % (chr(ord('a')+piece[1]-1),chr(ord('0')+piece[0]),capture,chr(ord('a')+piece[1]+f*i-1),chr(ord('0')+piece[0]+r*i))
                append_move(move_string) #add it to the list
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
              #shift as appropriate
              if(r > 0): new_square = new_square << (i<<3)    #leftshift by 8 bits per row
              elif(r < 0): new_square = new_square >> (i<<3)  #rightshift (i<<3 = i*8)
              if(f > 0): new_square = new_square << i     #<< by f
              elif(f < 0): new_square = new_square >> i   #>> by -f
              if(not (new_square & friendly_pieces)): #no piece conflict
                if(new_square & opponent_pieces + state.ep): capture = 'x' #piece captured
                else: capture = '-' #no capture
                #store the ouput string (N+old_location+capture+new_location)
                move_string = "B%s%s%s%s%s" % (chr(ord('a')+piece[1]-1),chr(ord('0')+piece[0]),capture,chr(ord('a')+piece[1]+f*i-1),chr(ord('0')+piece[0]+r*i))
                append_move(move_string) #add it to the list
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
      for r in (-1,0,1): #cycle through all avaliable angles of motion
        for f in (-1,0,1):
          for i in range (1,8): #queen can move up to 7 squares
            if(piece[0]+(r*i)>0 and piece[0]+(r*i)<=8 and piece[1]+(f*i)>0 and piece[1]+(f*i) <=8): #check board boundaries
              new_square = queen_square
              #shift as appropriate
              if(r > 0): new_square = new_square << (i<<3)    #leftshift by 8 bits per row (8*i = i<<3)
              elif(r < 0): new_square = new_square >> (i<<3)  #rightshift by 8*i
              if(f > 0): new_square = new_square << i     #<< by f
              elif(f < 0): new_square = new_square >> i   #>> by -f
              if(not (new_square & friendly_pieces)): #no piece conflict
                if(new_square & opponent_pieces + state.ep): capture = 'x' #piece captured
                else: capture = '-' #no capture
                #store the ouput string (N+old_location+capture+new_location)
                move_string = "Q%s%s%s%s%s" % (chr(ord('a')+piece[1]-1),chr(ord('0')+piece[0]),capture,chr(ord('a')+piece[1]+f*i-1),chr(ord('0')+piece[0]+r*i))
                append_move(move_string) #add it to the list
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
    for r in (-1,0,1): #king can move 1 space in any direction (+1,0,-1)
      for f in (-1,0,1):
        if(piece[0]+r>0 and piece[0]+r<=8 and piece[1]+f>0 and piece[1]+f <=8): #check board boundaries
          new_square = king_square  #storage for new location
          #shift as appropriate
          if(r > 0): new_square = new_square << 8    #leftshift by 8 bits per row
          elif(r < 0): new_square = new_square >> 8  #rightshift 
          if(f > 0): new_square = new_square << 1    #<< by f
          elif(f < 0): new_square = new_square >> 1  #>> by -f
          if(not (new_square & friendly_pieces)): #no piece conflict
            if(new_square & opponent_pieces + state.ep): capture = 'x' #piece captured
            else: capture = '-' #no capture
            #store the ouput string (N+old_location+capture+new_location)
            move_string = "K%s%s%s%s%s" % (chr(ord('a')+piece[1]-1),chr(ord('0')+piece[0]),capture,chr(ord('a')+piece[1]+f-1),chr(ord('0')+piece[0]+r))
            append_move(move_string) #add it to the list
            
  return gen_moves
  
#Boo
