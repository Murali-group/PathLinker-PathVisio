package pathlinker;

import org.osgi.framework.BundleActivator;
import org.osgi.framework.BundleContext;
import org.pathvisio.desktop.plugin.Plugin;

public class Activator implements BundleActivator {

    private PathLinker plugin;

    @Override
    public void start(BundleContext context) throws Exception {
       plugin = new PathLinker();
       context.registerService(Plugin.class.getName(), plugin, null);
    }

    @Override
    public void stop(BundleContext context) throws Exception {
       plugin.done();
    }

}
