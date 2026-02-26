/* @refresh reload */
import { render } from "solid-js/web";

import "../../style.css";
import RECO from "./reco";

render(() => <RECO/>, document.getElementById("root") as HTMLElement);