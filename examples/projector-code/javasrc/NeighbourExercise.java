/*
 * Copyright (c) 2009, Swedish Institute of Computer Science. All rights
 * reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer. 2. Redistributions in
 * binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or other
 * materials provided with the distribution. 3. Neither the name of the
 * Institute nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written
 * permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: GUI.java,v 1.142 2009/07/03 13:37:40 fros4943 Exp $
 */

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.GraphicsEnvironment;
import java.awt.Point;
import java.awt.Rectangle;
import java.awt.Stroke;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.ComponentAdapter;
import java.awt.event.ComponentEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;

import javax.swing.JDialog;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.Timer;

import java.awt.*;
import java.awt.event.*;

/**
 * Neighbour Exercise projector application.
 * Should be connected to a sensor node executing projector.c.
 * 
 * @author Fredrik Osterlind
 */
public class NeighbourExercise extends JPanel {
  private static final long serialVersionUID = -2082646601087328426L;

  public static final String SERIALDUMP_WINDOWS = "serialdump-windows.exe";
  public static final String SERIALDUMP_LINUX = "../../../tools/sky/serialdump-linux";

  private static Process serialDumpProcess;
  private static JFrame frame;

  private int PAINTED_NEIGHBOURS = 3;

  private ArrayList<Mote> motes = new ArrayList<Mote>();

  public NeighbourExercise(final String comPort) {
    setBackground(Color.WHITE);

    addComponentListener(new ComponentAdapter() {
      public void componentResized(ComponentEvent e) {
        for (Mote m: motes) {
          m.positionUnknown = true;
        }
      }
    });

    addMouseListener(new MouseAdapter() {
      public void mousePressed(MouseEvent e) {
        PAINTED_NEIGHBOURS++;
        if (PAINTED_NEIGHBOURS > 3) {
          PAINTED_NEIGHBOURS = 1;
        }
        frame.setTitle("Track Top " + PAINTED_NEIGHBOURS + "/3 Neighbour Exercise");
        repaint();
      }
    });

    new Thread(new Runnable() {
      public void run() {
        connectToCOMPort(comPort);
      }
    }).start();
  }

  private Point positionToScreen(int x, int y) {
    return new Point(
        (int) (BORDER_OFFSET + (double)x*(getWidth()-2*BORDER_OFFSET)/100.0),
        (int) (BORDER_OFFSET + (double)y*(getHeight()-4*BORDER_OFFSET)/100.0)
    );
  }
  public  static final int   BORDER_OFFSET = 35;
  private static final Color DESK_COLOR    = new Color(200, 120, 0);
  public void paintComponent(Graphics g) {
    super.paintComponent(g);

    ((Graphics2D)g).setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
    ((Graphics2D)g).setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);


    /* Background: coordinates and desk */
    g.setColor(Color.DARK_GRAY);
    g.drawString("(0,0)", 5, 20);
    g.drawString("(100,100)", getWidth()-60, getHeight()+3-2*BORDER_OFFSET);
    g.setColor(DESK_COLOR);
    g.fillRect(getWidth()/2-100, 20, 200, 70);
    g.setColor(Color.BLACK);
    g.drawRect(getWidth()/2-100, 20, 200, 70);

    /* Separator */
    g.setColor(Color.BLACK);
    g.drawLine(
        0, getHeight()-2*BORDER_OFFSET,
        getWidth(), getHeight()+5-2*BORDER_OFFSET);

    /* Update mote positions (unknowns are painted at bottom row) */
    int x = BORDER_OFFSET;
    for (Mote m: motes) {
      m.bottomPosition.x = x;
      m.bottomPosition.y = getHeight()-BORDER_OFFSET;
      x += 2*Mote.MOTE_SIZE;
    }

    /* Paint mote connections */
    Stroke old = ((Graphics2D)g).getStroke();
    Stroke connectionStroke = new BasicStroke(Mote.MOTE_SIZE/3);
    ((Graphics2D)g).setStroke(connectionStroke);
    for (Mote m: motes) {
      m.paintConnections(g);
    }
    ((Graphics2D)g).setStroke(old);

    /* Paint motes */
    for (Mote m: motes) {
      m.paintMote(g);
    }
  }

  private void connectToCOMPort(String comPort) {
    /* Connect to COM using external serialdump application */
    String osName = System.getProperty("os.name").toLowerCase();
    String fullCommand;
    if (osName.startsWith("win")) {
      fullCommand = SERIALDUMP_WINDOWS + " " + "-b115200" + " " + comPort;
    } else {
      fullCommand = SERIALDUMP_LINUX   + " " + "-b115200" + " " + comPort;
    }

    try {
      String[] cmd = fullCommand.split(" ");

      serialDumpProcess          = Runtime.getRuntime().exec(cmd);
      final BufferedReader input = new BufferedReader(new InputStreamReader(serialDumpProcess.getInputStream()));
      final BufferedReader err   = new BufferedReader(new InputStreamReader(serialDumpProcess.getErrorStream()));

      /* Start thread listening on stdout */
      Thread readInput = new Thread(new Runnable() {
        public void run() {
          String line;
          try {
            while ((line = input.readLine()) != null) {
              try {
                parseSerialData(line);
              } catch (RuntimeException e) {
                System.err.println("Parse error: " + e.getMessage());
                e.printStackTrace();
              }
            }
            input.close();
            System.out.println("Serialdump process shut down, exiting");
            System.exit(1);
          } catch (IOException e) {
            System.err.println("Exception when reading from serialdump");
            e.printStackTrace();
            System.exit(1);
          }
        }
      }, "read input stream thread");

      /* Start thread listening on stderr */
      Thread readError = new Thread(new Runnable() {
        public void run() {
          String line;
          try {
            while ((line = err.readLine()) != null) {
              System.err.println("Serialdump error stream> " + line);
            }
            err.close();
          } catch (IOException e) {
            System.err.println("Exception when reading from serialdump");
            e.printStackTrace();
            System.exit(1);
          }
        }
      }, "read error stream thread");

      readInput.start();
      readError.start();

    } catch (Exception e) {
      System.err.println("Exception when executing '" + fullCommand + "'");
      System.err.println("Exiting application");
      e.printStackTrace();
      System.exit(1);
    }

    System.out.println("Listening on COM port: " + comPort);
  }

  private static final int LINEPOS_ADDRESS        = 0;
  private static final int LINEPOS_TYPE           = 1;
  private static final int LINEPOS_POSITION       = 2;
  private static final int LINEPOS_MAG            = 3;
  private static final int LINEPOS_NEIGHBOUR1     = 2;
  private static final int LINEPOS_NEIGHBOUR1_LQI = 3;
  private static final int LINEPOS_NEIGHBOUR2     = 4;
  private static final int LINEPOS_NEIGHBOUR2_LQI = 5;
  private static final int LINEPOS_NEIGHBOUR3     = 6;
  private static final int LINEPOS_NEIGHBOUR3_LQI = 7;
  private static final int LINEPOS_ALARM_ADDRESS  = 2;

  private void parseMagData(Mote mote, String line) {
    /* Position */
    parsePosition(mote, line);

    String[] components = line.split(" ");
    String mag = components[LINEPOS_MAG];

    String posArr[] = mag.split("[\\[,\\]]");

    mote.mag_x = Integer.parseInt(posArr[1]);
    mote.mag_y = Integer.parseInt(posArr[2]);
    mote.mag_z = Integer.parseInt(posArr[3]);
  }

  private void parseReportData(Mote mote, String line) {
    /* Neighbours */
    parseNeighbours(mote, line);
  }

  private final static int POSITION_MIN = 1;
  private final static int POSITION_MAX = 100;
  private void parsePosition(Mote mote, String line) {
    String[] components = line.split(" ");

    String position = components[LINEPOS_POSITION];
    String posArr[] = position.split("[(,)]");
    int x = Integer.parseInt(posArr[1]);
    int y = Integer.parseInt(posArr[2]);

    if (x<POSITION_MIN || x>POSITION_MAX
     || y<POSITION_MIN || y>POSITION_MAX) {
      mote.positionUnknown = true;
      throw new RuntimeException("Bad position: (" + x + "," + y + ")");
    }

    Point tmp = positionToScreen(x, y);
    mote.positionUnknown = false;

    mote.position.x = tmp.x;
    mote.position.y = tmp.y;
  }

  private final static int LQI_MIN = 0x00;
  private final static int LQI_MAX = 0xFF;
  private void parseNeighbours(Mote mote, String line) {
    String[] components = line.split(" ");
    boolean found = false;

    // TODO FIXME
    System.out.println("Neighbours " + line);

    /* Neighbour 1 */
    String addr;
    String lqiStr;
    String lqiArr[];
    
    addr   = components[LINEPOS_NEIGHBOUR1];
    lqiStr = components[LINEPOS_NEIGHBOUR1_LQI]; 
    lqiArr = lqiStr.split("=");
    found  = false;
    mote.neighbour1highlight = false;
    for (Mote m: motes) {
      if (m.address.equals(addr)) {
        int lqi = Integer.parseInt(lqiArr[1]);
        lqi = lqi<LQI_MIN?LQI_MIN:lqi;
        lqi = lqi>LQI_MAX?LQI_MAX:lqi;

        if (mote.neighbour1 != m) {
          mote.neighbour1highlight = true;
        }
        found = true;
        mote.neighbour1 = m;
        mote.neighbour1LQI = (int)((double)(lqi-LQI_MIN)/(LQI_MAX-LQI_MIN));
        break;
      }
    }
    if (!found) {
      mote.neighbour1 = null;
    }

    /* Neighbour 2 */
    addr   = components[LINEPOS_NEIGHBOUR2];
    lqiStr = components[LINEPOS_NEIGHBOUR2_LQI]; 
    lqiArr = lqiStr.split("=");
    found  = false;
    mote.neighbour2highlight = false;
    for (Mote m: motes) {
      if (m.address.equals(addr)) {
        int lqi = Integer.parseInt(lqiArr[1]);
        lqi = lqi<LQI_MIN?LQI_MIN:lqi;
        lqi = lqi>LQI_MAX?LQI_MAX:lqi;


        if (mote.neighbour2 != m) {
          mote.neighbour2highlight = true;
        }
        found = true;
        mote.neighbour2 = m;
        mote.neighbour2LQI = (int)((double)(lqi-LQI_MIN)/(LQI_MAX-LQI_MIN));
        break;
      }
    }
    if (!found) {
      mote.neighbour2 = null;
    }

    /* Neighbour 3 */
    addr   = components[LINEPOS_NEIGHBOUR3];
    lqiStr = components[LINEPOS_NEIGHBOUR3_LQI]; 
    lqiArr = lqiStr.split("=");
    found  = false;
    mote.neighbour3highlight = false;
    for (Mote m: motes) {
      if (m.address.equals(addr)) {
        int lqi = Integer.parseInt(lqiArr[1]);
        lqi = lqi<LQI_MIN?LQI_MIN:lqi;
        lqi = lqi>LQI_MAX?LQI_MAX:lqi;


        if (mote.neighbour3 != m) {
          mote.neighbour3highlight = true;
        }
        found = true;
        mote.neighbour3 = m;
        mote.neighbour3LQI = (int)((double)(lqi-LQI_MIN)/(LQI_MAX-LQI_MIN));
        break;
      }
    }
    if (!found) {
      mote.neighbour3 = null;
    }

  }

  private void parseSerialData(String line) {
    System.out.println("SERIALDATA> " + line);

    if (line == null) {
      return;
    }
    String[] components = line.split(" ");

    String address = components[LINEPOS_ADDRESS];
    if (address.length()==0) {
      return;
    }

    if (address.endsWith(":")) {
      address = address.replace(":", "");
    }

    /* Locate mote, or create new */
    Mote mote = null;
    for (Mote m: motes) {
      if (m.address.equals(address)) {
        mote = m;
        break;
      }
    }
    if (mote == null) {
      if (address.length() != 23 || !address.contains(":")) {
        throw new RuntimeException("Bad address: " + address);
      }
      mote = new Mote(address);
      motes.add(mote);
    }

    /* Parse type */
    String type = components[LINEPOS_TYPE];
    if (type.equals("MAG")) {
      parseMagData(mote, line);
    } else if (type.equals("REPORT")) {
      parseReportData(mote, line);
    } else {
      throw new RuntimeException("Unknown type: " + type);
    }

    mote.highlight();
    repaint();
  }

  public static void main(final String[] args) {
    if (args.length != 1) {
      System.err.println("Usage: java NeighbourExercise /dev/com1");
      System.err.println("Usage: ant run -Dargs=\"/dev/com1\"");
      return;
    }

    final String comPort = args[0];
    javax.swing.SwingUtilities.invokeLater(new Runnable() {
      public void run() {
        /* Make sure we have nice window decorations */
        JFrame.setDefaultLookAndFeelDecorated(true);
        JDialog.setDefaultLookAndFeelDecorated(true);

        NeighbourExercise demo = new NeighbourExercise(comPort);

        /* Frame */
        frame = new JFrame("Track Top 3/3 Neighbour Exercise");
        Rectangle maxSize = 
          GraphicsEnvironment.getLocalGraphicsEnvironment().getMaximumWindowBounds();
        if (maxSize != null) {
          frame.setMaximizedBounds(maxSize);
        }
        frame.setSize(1024-50, 768-50);
        frame.setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE);
        frame.addWindowListener(new WindowAdapter() {
          public void windowClosing(WindowEvent e) {
            if (serialDumpProcess != null) {
              try {
                serialDumpProcess.destroy();
              } catch (Exception ex) {
              }
            }
            System.exit(0);
          }
        });
        frame.setContentPane(demo);
        frame.setVisible(true);
      }
    });
  }

  private class Mote {
    public static final int MOTE_SIZE = 20;
    public static final int MIN_MAG = -2048; /* Range of HMC5883 output 0xF800 */
    public static final int MAX_MAG =  2047; /* Range of HMC5883 output 0x07FF */

    Mote neighbour1             = null;
    int  neighbour1LQI          = 0;
    boolean neighbour1highlight = false;
    Mote neighbour2             = null;
    double neighbour2LQI        = 0;
    boolean neighbour2highlight = false;
    Mote neighbour3             = null;
    double neighbour3LQI        = 0;
    boolean neighbour3highlight = false;

    int mag_x                   = 0;
    int mag_y                   = 0;
    int mag_z                   = 0;

    boolean positionUnknown   = true;
    Point position            = new Point(0, 0);
    Point bottomPosition      = new Point(0, 0);

    private String address;
    private boolean highlighted = false;
    private Timer highlightTimer  = new Timer(200, new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        highlighted         = false;
        neighbour1highlight = false;
        neighbour2highlight = false;
        neighbour3highlight = false;
        highlightTimer.stop();
        repaint();
      }
    });

    public Mote(String address) {
      this.address = address;
    }

    public void highlight() {
      if (highlightTimer.isRunning()) {
        highlightTimer.stop();
      }
      highlighted = true;
      highlightTimer.start();
      repaint();
    }

    private final Color COLOR_CONN1 = new Color(100, 100, 100, 150);
    private final Color COLOR_CONN2 = new Color(100, 100, 100, 100);
    private final Color COLOR_CONN3 = new Color(100, 100, 100, 50);
    private final Color COLOR_CONN_HIGHLIGHT = new Color(0, 255, 0, 100);
    public void paintConnections(Graphics g, Mote mote, double lqi) {

      g.drawLine(
          position.x, position.y, 
          mote.position.x, mote.position.y );

      Point centrePoint = new Point(
          (3*mote.position.x + position.x)/4,
          (3*mote.position.y + position.y)/4 );

      int startAngle = (int) (-180 * Math.atan2(
          mote.position.y - position.y, mote.position.x - position.x)/Math.PI - 90);

      //g.setColor(Color.BLACK);
      g.drawArc(centrePoint.x-5, centrePoint.y-5, 10, 10, startAngle, 180);
    }

    public void paintConnections(Graphics g) {
      if (positionUnknown) {
        return;
      }
      if (PAINTED_NEIGHBOURS < 1) {
        return;
      }
      if (neighbour1 != null && !neighbour1.positionUnknown) {
        g.setColor(COLOR_CONN1);
        if (neighbour1highlight) {
          g.setColor(COLOR_CONN_HIGHLIGHT);
        }
        paintConnections(g, neighbour1, neighbour1LQI);
      }
      if (PAINTED_NEIGHBOURS < 2) {
        return;
      }
      if (neighbour2 != null && !neighbour2.positionUnknown) {
        g.setColor(COLOR_CONN2);
        if (neighbour2highlight) {
          g.setColor(COLOR_CONN_HIGHLIGHT);
        }
        paintConnections(g, neighbour2, neighbour2LQI);
      }
      if (PAINTED_NEIGHBOURS < 3) {
        return;
      }
      if (neighbour3 != null && !neighbour3.positionUnknown) {
        g.setColor(COLOR_CONN3);
        if (neighbour3highlight) {
          g.setColor(COLOR_CONN_HIGHLIGHT);
        }
        paintConnections(g, neighbour3, neighbour3LQI);
      }
    }

    public void paintMote(Graphics g) {

      /* Mote highlight (bottom) */
      if (highlighted) {
        g.setColor(Color.GREEN);
        g.fillOval(bottomPosition.x-MOTE_SIZE, bottomPosition.y-MOTE_SIZE, MOTE_SIZE*2, MOTE_SIZE*2);
      }

      /* Mote mag (color) */
      if (!positionUnknown) {
        /* Mote highlight */
        if (highlighted) {
          g.setColor(Color.GREEN);
          g.fillOval(position.x-MOTE_SIZE, position.y-MOTE_SIZE, MOTE_SIZE*2, MOTE_SIZE*2);
        }

        if (mag_x != 0 && mag_y != 0 && mag_z != 0) {
          int l = mag_x;
          if (l > Mote.MAX_MAG) {
            l = Mote.MAX_MAG;
          } else if (l < Mote.MIN_MAG) {
            l = Mote.MIN_MAG;
          }
          float val_x = (float)(l-MIN_MAG)/(float)(MAX_MAG-MIN_MAG);

          l = mag_y;
          if (l > Mote.MAX_MAG) {
            l = Mote.MAX_MAG;
          } else if (l < Mote.MIN_MAG) {
            l = Mote.MIN_MAG;
          }
          float val_y = (float)(l-MIN_MAG)/(float)(MAX_MAG-MIN_MAG);

          l = mag_z;
          if (l > Mote.MAX_MAG) {
            l = Mote.MAX_MAG;
          } else if (l < Mote.MIN_MAG) {
            l = Mote.MIN_MAG;
          }
          float val_z = (float)(l-MIN_MAG)/(float)(MAX_MAG-MIN_MAG);

          g.setColor(new Color(val_x, val_y, val_z));
        } else {
          g.setColor(Color.CYAN);
        }
        g.fillOval(position.x-MOTE_SIZE/2, position.y-MOTE_SIZE/2, MOTE_SIZE, MOTE_SIZE);

        /* Mote outline */
        g.setColor(Color.BLACK);
        g.drawOval(position.x-MOTE_SIZE/2, position.y-MOTE_SIZE/2, MOTE_SIZE, MOTE_SIZE);
      }

      /* Mote outline (lower) */
      g.setColor(Color.BLACK);
      g.drawOval(bottomPosition.x-MOTE_SIZE/2, bottomPosition.y-MOTE_SIZE/2, MOTE_SIZE, MOTE_SIZE);

      if (!positionUnknown) {
        /* Mote mag (string) */
        if (mag_x != 0 && mag_y != 0 && mag_z != 0) {
          String magString = "" + mag_x + "," + mag_y + "," + mag_z;
          g.drawString(magString,
              position.x - g.getFontMetrics().stringWidth(magString)/2,
              position.y-MOTE_SIZE/2-5);
        }
      }

      if (!positionUnknown) {
        /* Mote address */
        g.drawString(address,
            position.x - g.getFontMetrics().stringWidth(address)/2,
            position.y+MOTE_SIZE/2+g.getFontMetrics().getHeight());
      }

      /* Mote address (lower) */
      g.drawString(address,
          bottomPosition.x - g.getFontMetrics().stringWidth(address)/2,
          bottomPosition.y+MOTE_SIZE/2+g.getFontMetrics().getHeight());
    }

    public String toString() {
      return address;
    }
  }

}
