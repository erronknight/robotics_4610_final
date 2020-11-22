#!/usr/bin/env ruby

require 'erb'

class MazeGen
  def initialize()
    @wall_ii = 0
  end

  def next_ii
    i0 = @wall_ii
    @wall_ii += 1
    i0
  end

  def wall(xx, yy, rot=false)
    aa = rot ? 0 : (Math::PI/2)
    aa += 0.1 * (rand(7) - 3)
    %Q{
    <include>
      <uri>model://brick_box_3x1x3</uri>
      <name>wall#{(next_ii)}</name>
      <pose>#{xx} #{yy} -2.0 0 0 #{aa}</pose>
    </include>
    }
  end

  def roll
    1 + rand(4)
  end

  def safe(xx, yy)
    yy.abs > 2 || (20 - xx.abs).abs > 2
  end

  def split_y(walls, x0, y0, x1, y1)
    dx = x1 - x0
    dy = y1 - y0

    if (dy < 10) then
      return
    end

    wx = x0 + 6 + rand(dx - 6)
    wy = y0 + 6 + rand(dy - 6)

    jj = wy
    x0.step(x1+2, 3) do |ii|
      if ((ii - wx).abs >= 3) && safe(ii, jj) then
        walls.push(wall(ii, jj, true))
      end
    end

    split_x(walls, x0, y0, x1, wy - 2)
    split_x(walls, x0, wy + 2, x1, y1)
  end

  def split_x(walls, x0, y0, x1, y1)
    dx = x1 - x0
    dy = y1 - y0

    if (dx < 10) then
      return
    end

    wx = x0 + 6 + rand(dx - 6)
    wy = y0 + 6 + rand(dy - 6)

    ii = wx
    y0.step(y1+2, 3) do |jj|
      if ((jj - wy).abs >= 3) && safe(ii, jj) then
        walls.push(wall(ii, jj))
      end
    end

    split_y(walls, x0, y0, wx - 2, y1)
    split_y(walls, wx + 2, y0, x1, y1)
  end

  def gen
    tpl = ERB.new(File.read("template.xml.erb"), trim_mode: "-<>")
    walls = []
    (-9).upto(9) do |ii|
      xx = ii * 3
      walls.push(wall(xx, -28, true))
      walls.push(wall(xx, 28, true))
      walls.push(wall(-28, xx, false))
      walls.push(wall(28, xx, false))
    end

    split_x(walls, -27, -27, 27, 27)

    puts tpl.run(binding)
  end
end

mg = MazeGen.new
mg.gen
