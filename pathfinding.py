from collections import deque
import os
from random import randrange
from time import sleep

def main():
  Grid()
  graph = Grid.graph
  start = Grid.start
  target = Grid.target
  pathfinding = PathFinding( graph, Grid.max_columns, Grid.max_columns )

  traversal = input( 'Choose a graph traversal:\n1 - DFS\n2 - BFS\n3 - Dijkstra\n4 - A* search\n' )
  match traversal:
    case '1':
      print('dfs')
      pathfinding.dfs( start )
    case '2':
      pathfinding.bfs( *start )
    case '3':
      pathfinding.dijkstra( *start )
    case _:
      Grid.transform_in_a_star_vertex()
      pathfinding.a_star( start, target )

class Color:
  CYAN = '\033[96m'
  DARKCYAN = '\033[36m'
  PINK = '\033[95m'
  BLUE = '\033[94m'
  YELLOW = '\033[93m'
  GREEN = '\033[92m'
  RED = '\033[91m'
  BOLD = '\033[1m'
  UNDERLINE = '\033[4m'
  END = '\033[0m'
  INVISIBLE = '\033[08m'
  BORDER = '▀'

class Vertex:
  def __init__( self, position: tuple, is_target=False, is_start=False ) -> None:
    self.position = position
    self.value = 1
    self.target = is_target
    self.start = is_start
    self.visited = False
    self.path = False
    self.previous = None
  
  def __str__( self ) -> str:
    if self.start:
      return f'{ Color.BORDER }'.join( ( Color.PINK, Color.END ) )
    if self.target:
      return f'{ Color.BORDER }'.join( ( Color.YELLOW, Color.END ) )
    if self.path:
      return f'{ Color.BORDER }'.join( ( Color.GREEN, Color.END ) )
    if self.visited:
      return f'{ Color.BORDER }'.join( ( Color.BLUE, Color.END ) )
    else: 
      return f" ".join( ( Color.INVISIBLE, Color.END ) )
    
class A_star_vertex( Vertex ):
  """
  f(n) most optimal path from the start node to the goal node\n
  g(n) is the cost of the path from the start node to n\n
  h(n) is a heuristic function that estimates the cost of the cheapest path from n to the goal\n
  """
  def __init__(self, position: tuple, is_target=False, is_start=False) -> None:
    super().__init__(position, is_target, is_start)
    self.g = float( 'inf' )
    self.h = float( 'inf' )
    self.f: None | int = None
    self.explored = False
  
  def __str__(self) -> str:
    if self.explored:
      return f'{ Color.BORDER }'.join( ( Color.RED, Color.END ) )
    return super().__str__()

class PathFinding:
  def __init__( self, graph, rows, columns ) -> None:
    self.MAX_ROWS = rows
    self.MAX_COLUMNS = columns
    self.graph = graph
  
  
  def dfs( self, start: tuple ) -> None:
    def shortest_path():
      while path:
        row, column = path.popleft()
        self.graph[row][column].path = True
        Grid.print_grid()

    def explore( row, column ):
      if Grid.is_border( row, column ) or self.graph[row][column].visited:
        return
      if self.graph[row][column].target:
        return True 
      
      path.append( ( row, column ) )
      self.graph[row][column].visited = True
      
      Grid.print_grid()

      result = (
        explore( row - 1, column ) or
        explore( row, column + 1 ) or
        explore( row + 1, column ) or
        explore( row, column - 1 ) 
      )
      return result
    
    path: tuple = deque( [] )
    explore( *start )
    
    shortest_path()
  

  def bfs( self, row, column ) -> None:
    def explore( row, column ):
      if Grid.is_border( row, column ) or self.graph[row][column].visited: 
        return

      neighbor = self.graph[row][column]
      queue.append( neighbor )
      neighbor.previous = vertex

    target = None
    start = self.graph[row][column]
    queue: list[Vertex] = deque( [ start ] )
    while queue:
      vertex = queue.popleft()
      row, column = vertex.position

      if vertex.target: 
        target = vertex
        break
      if vertex.visited: 
        continue

      self.graph[row][column].visited = True

      Grid.print_grid()

      # explore neighbors
      left: tuple = ( row, column - 1 )
      right: tuple = ( row, column + 1 )
      up: tuple = ( row - 1, column )
      bottom: tuple = ( row + 1, column )

      explore( *left )
      explore( *right )
      explore( *up )
      explore( *bottom )
    
    PathFinding.shortest_path( target )
  

  def dijkstra( self, row, column ) -> None:

    def calculate_distance_of_neighbor_from_the_start_vertex( neighbor: tuple, vertex ):
      if not Grid.is_border( *neighbor ) and neighbor in unvisited:
        row, column = neighbor
        neighbor = self.graph[row][column]
        distance = shortest_distance[vertex] + neighbor.value
        if distance < shortest_distance.get( neighbor, float( 'inf' ) ):
          shortest_distance[neighbor] = distance
          neighbor.previous = vertex
    
    #TODO: MINHEAP/PRIORITY QUEUE
    def get_shortest_distance_from_start( shortest_distance: dict[Vertex, int], unvisited: set[tuple] ) -> Vertex:
      minimum_distance = float( 'inf' )
      for vertex, distance in shortest_distance.items():
        if vertex.position in unvisited and distance < minimum_distance:
          minimum_distance = distance
          minimum_vertex = vertex
      return minimum_vertex

    start = self.graph[row][column]
    target = None
    shortest_distance = { start: 0 }
    unvisited: set[tuple] = { ( row, column ) for row in range( self.MAX_ROWS ) for column in range( self.MAX_COLUMNS ) if not Grid.is_border( row, column ) }
    while unvisited:
      vertex = get_shortest_distance_from_start( shortest_distance, unvisited )
      row, column = vertex.position

      if vertex.target: 
        target = self.graph[row][column]
        break

      # explore neighbors
      left: tuple = ( row, column - 1 )
      right: tuple = ( row, column + 1 )
      up: tuple = ( row - 1, column )
      bottom: tuple = ( row + 1, column )
      
      calculate_distance_of_neighbor_from_the_start_vertex( up, vertex )
      calculate_distance_of_neighbor_from_the_start_vertex( right, vertex )
      calculate_distance_of_neighbor_from_the_start_vertex( left, vertex )
      calculate_distance_of_neighbor_from_the_start_vertex( bottom, vertex )

      unvisited.remove( vertex.position )
      self.graph[row][column].visited = True
      Grid.print_grid()
    
    PathFinding.shortest_path( target )


  def a_star( self, start: tuple, target: tuple ) -> None:
    def heuristic( vertex_position: tuple, target_position: tuple ) -> int: 
      """uses manhattan distance to estimates how far it is from vertex to the target"""
      return abs( vertex_position[0] - target_position[0] ) + abs( vertex_position[1] - target_position[1] )

    def lowest_f_value() -> Vertex:
      min_f = float( 'inf' )
      for node in open_set:
        if node.f < min_f: 
          min_f = node.f
          vertex = node
        elif node.f == vertex.f and node.h < vertex.h:
          vertex = node
      return vertex

    def shortest_path( vertex ) -> None:
      path = []
      while vertex:
        path.append( vertex )
        vertex = vertex.previous

      while path:  
        vertex = path.pop()
        vertex.path = True
        Grid.print_grid()

    def calculate_distance( row, column ) -> None:
      if Grid.is_border( row, column ): return
      
      neighbor: Vertex = self.graph[row][column]
      if neighbor.visited == False:
        g_cost = vertex.g + neighbor.value
        h_cost = heuristic( neighbor.position, target )
        f_cost = g_cost + h_cost
        # it will never recauculate cause neighbors have the same length
        if neighbor.f == None or f_cost < neighbor.f:
          neighbor.g = g_cost
          neighbor.h = h_cost
          neighbor.f = f_cost
          neighbor.previous = vertex
          open_set.add( neighbor ) # when change set for priority queue, only add if it is not in queue

    start = A_star_vertex( start, is_start=True )
    start.g = 0
    start.h = heuristic( start.position, target )
    start.f = start.h

    open_set: set[Vertex] = { start } # TODO: use priority queue
    while open_set:
      vertex = lowest_f_value()

      if vertex.position == target:
        return shortest_path( vertex.previous )

      row, column = vertex.position

      left: tuple = ( row, column - 1 )
      right: tuple = ( row, column + 1 )
      up: tuple = ( row - 1, column )
      bottom: tuple = ( row + 1, column )

      calculate_distance( *left )
      calculate_distance( *right )
      calculate_distance( *up )
      calculate_distance( *bottom )
      
      open_set.remove( vertex )
      vertex.visited = True
      Grid.print_grid()

  @staticmethod
  def shortest_path( vertex: Vertex ) -> None:
    path = []
    while vertex:
      path.append( vertex )
      vertex = vertex.previous

    while path:  
      vertex = path.pop()
      vertex.path = True
      Grid.print_grid()

class Grid:
  DELAY_SECONDS = 0.03
  max_rows = None
  max_columns = None
  graph = None
  target = None
  start = None

  def __init__( self ) -> None:
    self.init_grid()
  
  def init_grid( self ):
    Grid.max_rows, Grid.max_columns = Grid.get_graph_dimensions()
    Grid.make_grid()

    row, column = target = Grid.get_target()
    Grid.graph[row][column] = Vertex( target, is_target = True )
    Grid.print_grid( animation = True )

    row, column = start = Grid.get_start()
    Grid.graph[row][column] = Vertex( start, is_start = True )

    Grid.target = target
    Grid.start = start

  @staticmethod
  def make_grid():
    Grid.graph = [
      [ 
        Color.BORDER 
        if Grid.is_border( row, column ) else Vertex( ( row, column ) ) 
        for column in range( Grid.max_columns )
      ] 
      for row in range( Grid.max_rows ) 
    ] 

  @staticmethod
  def get_graph_dimensions() -> tuple[int, int]:
    MINIMUM_AREA = 16
    inputs = ( 'Number of rows: ', 'Number of columns: ' )
    valid_inputs, values = 0, []
    while valid_inputs < len( inputs ):
      os.system( 'cls' if os.name == 'nt' else 'clear' )
      
      for value in values:
        print( inputs[valid_inputs - 1] + str( value ) )

      try:
        value = int( input( inputs[valid_inputs] ) )
        if value <= 0: raise ValueError
      except: 
        continue

      values.append( value )
      valid_inputs += 1
      
      if valid_inputs == len( inputs ) and values[0] * values[1] < MINIMUM_AREA:
        valid_inputs = 0
        values.clear()
        print( "Graph area too small... Don't be afraid honey" )
        sleep(3.3)
    return values
  
  @staticmethod
  def get_start() -> tuple[int, int]:
    while True:
      try:
        row = int( input( 'start row: ' ) )
        column = int( input( 'start column: ' ) )
        if row < 0: 
          row = Grid.max_rows - 1 + row 
        if column < 0: 
          column = Grid.max_columns - 1 + column 
        if Grid.graph[row][column] == Color.BORDER: raise ValueError
        if ( row, column ) == Grid.target: raise ValueError
        return ( row, column )
      except:
        print( 'Values must be integers and position not occupied by border or target' )
        continue

  @staticmethod
  def get_target() -> tuple[int, int]:
    return randrange( 1, Grid.max_rows - 1 ), randrange( 1, Grid.max_columns - 1 )

  @staticmethod
  def is_border( row, column ) -> bool:
    return (
      row <= 0 
      or column <= 0 
      or row >= Grid.max_rows - 1 
      or column >= Grid.max_columns - 1
      or ( Grid.graph and Grid.graph[row][column] == Color.BORDER )
    )
  
  @staticmethod
  def print_grid( animation: bool = False ) -> None:
    os.system( 'cls' if os.name == 'nt' else 'clear' )
    # sys.stdout.write("\033[H")
    # sys.stdout.flush()
    if animation:
      for row in Grid.graph:
        for element in row:
          print( element, end=' ', flush=True )
          if element == Color.BORDER: sleep( Grid.DELAY_SECONDS )
        print()
    else:
      for row in Grid.graph:
        for element in row:
          print( element, end=' ' )
        print()
    sleep( Grid.DELAY_SECONDS )

  @staticmethod
  def transform_in_a_star_vertex():
    for row in range( Grid.max_rows ):
      for column in range( Grid.max_columns ):
        if Grid.is_border( row, column ): 
          continue
        elif ( row, column ) == Grid.start:
          Grid.graph[row][column] = A_star_vertex( ( row, column ), is_start = True )
        elif ( row, column ) == Grid.target:
          Grid.graph[row][column] = A_star_vertex( ( row, column ), is_target = True )
        else:
          Grid.graph[row][column] = A_star_vertex( ( row, column ) )

main()  