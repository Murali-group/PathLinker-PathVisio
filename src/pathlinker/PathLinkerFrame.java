package pathlinker;

import com.jgoodies.forms.layout.CellConstraints;
import com.jgoodies.forms.layout.FormLayout;
import java.awt.Component;
import java.awt.Frame;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.io.File;
import javax.swing.JButton;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JPopupMenu;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;
import javax.swing.JTextField;
import org.pathvisio.gui.dialogs.OkCancelDialog;

public class PathLinkerFrame extends OkCancelDialog {
    private final PathLinker pathlinker;
    private JTextField searchText;
    private JTextArea sourceNamesText;
    private JTextArea targetNamesText;

    public PathLinkerFrame(JFrame parent, PathLinker path) {
        super(parent, "PathLinker", parent, true);
        pathlinker = path;
        setDialogComponent(createDialogPane());
        setSize(500, 700);
    }

    protected Component createDialogPane() {
        FormLayout layout = new FormLayout (
                "pref, 4dlu, 150dlu, 4dlu, min",
                "40dlu, 1dlu, 20dlu, 1dlu, 100dlu");
        JPanel panel = new JPanel(layout);
        CellConstraints cc = new CellConstraints();

        JLabel searchSource = new JLabel("Background Network ");
        searchText = new JTextField();
        final JButton browseButton = new JButton("Browse ");
        final JFileChooser fc = new JFileChooser();

        browseButton.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent e) {
                if (e.getSource() == browseButton) {
                    int returnVal = fc.showOpenDialog(PathLinkerFrame.this);
                    if (returnVal == JFileChooser.APPROVE_OPTION) {
                        File file = fc.getSelectedFile();
                        searchText.setText(file.toString());
                    } else {
                        searchText.setText("");
                    }
               }
            }
        });
        //background graph input
        panel.add(searchSource, cc.xy(1, 1));
        panel.add(searchText, cc.xy(3, 1));
        panel.add(browseButton, cc.xy(5, 1));

        //source nodes
        JLabel sources = new JLabel("<html> Enter source nodes(will edit) <br> " +
                "(source1 [tab] source2 [tab] ... sourceN) </html>");
        sourceNamesText = new JTextArea(7,10);
        JScrollPane scrollingAreaSource = new JScrollPane(sourceNamesText);
        panel.add(sources,cc.xy(3, 3));
        panel.add(scrollingAreaSource , cc.xy(3, 5));

        //target nodes
        JLabel targets = new JLabel("<html> Enter target nodes(will edit) <br> " +
            "(target1 [tab] target2 [tab] ... targetN) </html>");
        targetNamesText = new JTextArea(7,10);
        JScrollPane scrollingAreaTarget = new JScrollPane(sourceNamesText);
        panel.add(targets,cc.xy(3, 7));
        panel.add(scrollingAreaTarget , cc.xy(3, 9));

        return panel;
    }

}
