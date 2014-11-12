Chess Ai:
import chess_ai (in file chess_ai.py)
(optional) call set_meta_vals(depth, threads) to change execution parameters
call get_chess_move(FEN_board, ai_color) to get the move
-Function inputs are the FEN board string, and the AI color (true=white)
-Function returns a dictionary: {move, is_check}
 --Move will be a string move (Logical Chess Notaton)
 --Check will be either None, 'Check', or 'Checkmate'


