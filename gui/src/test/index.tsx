/* @refresh reload */
import { render } from "solid-js/web";

import "../style.css";
import Test from "./Test";

render(() => <Test/>, document.getElementById("root") as HTMLElement);
