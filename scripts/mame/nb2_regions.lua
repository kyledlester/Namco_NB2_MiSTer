-- Dump every ROM region MAME assembled for the running set (MAME 0.289) to NB2_OUT/<tag>.bin, so the MRA tool's
-- stream can be compared byte for byte with MAME's region memory. ROM-derived output: keep it outside the repo.
local M = manager.machine
local OUT = assert(os.getenv("NB2_OUT"))
for tag, r in pairs(M.memory.regions) do
  local name = tag:gsub("^:", ""):gsub(":", "_")
  local f = assert(io.open(OUT .. "/" .. name .. ".bin", "wb"))
  local n = r.size
  local chunk = {}
  for i = 0, n - 1 do
    chunk[#chunk + 1] = string.char(r:read_u8(i))
    if #chunk == 65536 then f:write(table.concat(chunk)); chunk = {} end
  end
  f:write(table.concat(chunk))
  f:close()
  print(string.format("%s %d", name, n))
end
M:exit()
