local MiniCompletion = require("mini.completion")
MiniCompletion.setup({
    lsp_completion = { auto_setup = true },
})

local MiniIcons = require("mini.icons")
MiniIcons.setup()

local MiniNotify = require("mini.notify")
MiniNotify.setup({
    content = {
        format = function(notif)
            return notif.msg
        end,
    },
})

local MiniSnippets = require("mini.snippets")
MiniSnippets.setup({
    snippets = {
        MiniSnippets.gen_loader.from_lang(),
    },
})
MiniSnippets.start_lsp_server({ match = false })
