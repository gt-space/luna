/* @refresh reload */
import { render } from "solid-js/web";

import "../../style.css";
import RecoFlasher from "./reco_flasher";

render(() => <RecoFlasher/>, document.getElementById("root") as HTMLElement);