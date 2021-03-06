package pathlinker;

import java.awt.event.ActionEvent;
import javax.swing.AbstractAction;
import org.pathvisio.desktop.PvDesktop;
import org.pathvisio.desktop.plugin.Plugin;

public class PathLinker implements Plugin {
    private final PathLinkerAction pathLinkerAction = new PathLinkerAction();
    PvDesktop desktop;
    @Override
    public void done() {
        desktop.unregisterMenuAction("Plugins", pathLinkerAction);
    }


    @Override
    public void init(PvDesktop desk) {

        this.desktop = desk;
        desktop.registerMenuAction ("Plugins", pathLinkerAction);
    }

private class PathLinkerAction extends AbstractAction {
    PathLinkerAction(){
            // The NAME property of an action is used as
            // the label of the menu item
            putValue (NAME, "PathLinker");
    }
    @Override
    public void actionPerformed(ActionEvent arg0) {
        PathLinkerFrame pathLinkerFrame = new PathLinkerFrame(desktop.getFrame(), PathLinker.this);
        pathLinkerFrame.setVisible(true);
    }


    }

}
