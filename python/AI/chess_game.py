import chess_ai

def play_game():
  player_color = raw_input('Chose your color (w/b): ') == 'w'
  
  if(raw_input('Use default settings? (y/n): ') == 'n'):
    depth = int(raw_input('Max Depth: '))
    threads = int(raw_input('Max Threads: '))
    chess_ai.set_meta_vals(depth, threads)
  else: reload(chess_ai)  # otherwise reload defaults
  
  # inital board state
  board = [['R','N','B','Q','K','B','N','R'],
    ['P','P','P','P','P','P','P','P'],
    ['1','1','1','1','1','1','1','1'],
    ['1','1','1','1','1','1','1','1'],        
    ['1','1','1','1','1','1','1','1'],
    ['1','1','1','1','1','1','1','1'],
    ['p','p','p','p','p','p','p','p'],
    ['r','n','b','q','k','b','n','r']]                    
    
  white_turn = True # white goes first
  ep = '-' # no En Passant, ever
  castle = 0 # players are not allowed to castle (too hard)
  
  print 'Enter moves like this: Ka1-a2'
  print 'Irregardless of color, all pieces are capitol, all ranks are lowercase'
  print 'For captures, use Ka1xa2'
  print 'Dont use P for pawns, ie a2-a4'
  print 'Append C or M for check/mate: a2-a4C'
    
  while(True):  # loop literally forever
    if(white_turn == player_color): # player turn
      print_board(board, player_color)
      move = raw_input('Enter a move: ')
    else: # ai turn
      long_move = chess_ai.get_chess_move(get_fen(board, white_turn, ep, castle),not player_color)
      print long_move # Debug
      check_check = long_move['check'] # check condition is second return value
      move = long_move['move'] # first return is move
    
    # preform the move
    if(move[0] >= 'A' and move[0] <= 'Z'):  # not a pawn
      board[ord(move[2])-ord('0')-1][ord(move[1])-ord('a')] = '1' # remove old piece
      if(white_turn): # is white, upper is fine
        board[ord(move[5])-ord('0')-1][ord(move[4])-ord('a')] = move[0]
      else: # is black, convert to lower case
        board[ord(move[5])-ord('0')-1][ord(move[4])-ord('a')] = chr(ord(move[0]) - ord('A') + ord('a'))
    else: # piece is pawn
      board[ord(move[1])-ord('0')-1][ord(move[0])-ord('a')] = '1' # remove old piece
      if(white_turn): # is white, upper is fine
        board[ord(move[4])-ord('0')-1][ord(move[3])-ord('a')] = 'P'
      else: # is black, convert to lower case
        board[ord(move[4])-ord('0')-1][ord(move[3])-ord('a')] = 'p'
        
    white_turn = not white_turn   # take turns
        

# print the current board state (rotate for player)        
def print_board(board, color):
  # flip the board as needed
  if(color): rows = reversed(range(0,8))
  else: rows = range(0,8)
  
  # print the things
  for row in rows:
    if(color): cols = range(0,8)
    else: cols = reversed(range(0,8))
    row_string = chr(ord('0') + row + 1) + ': '
    for col in cols:
      if(board[row][col] == '1'): row_string += '* '
      else: row_string += board[row][col] + ' '
    print row_string
  if(color): print '   a-b-c-d-e-f-g-h'
  else: print '   h-g-f-e-d-c-b-a'


# get fen notation
def get_fen(board, white_turn, ep, castle):
  fen = ''
  for row in range(0,8):
    for piece in board[row]:
      fen += piece      
    if(row != 7): fen += '/'
    else: fen += ' '
  
  if(white_turn): fen += 'w '
  else: fen += 'b '
  
  if(castle & 0x1): fen += 'K'
  if(castle & 0x2): fen += 'Q'
  if(castle & 0x4): fen += 'k'
  if(castle & 0x8): fen += 'q'
  
  fen += ' ' + ep
  return fen

