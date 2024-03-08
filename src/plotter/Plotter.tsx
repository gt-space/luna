import Footer from "../general-components/Footer";
import { GeneralTitleBar } from "../general-components/TitleBar";
import PlotterView from "./PlotterView";

function Plotter() {
    return <div class="window-template">
    <div style="height: 60px">
      <GeneralTitleBar name="Plotter"/>
    </div>
    <div style="display: flex; flex-direction: column; overflow: hidden">
      <PlotterView />
    </div>
    <div>
      <Footer/>
    </div>
</div>
}

export default Plotter;