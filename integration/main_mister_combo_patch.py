def patch(p, pairs):
    s=open(p).read()
    for a,b in pairs:
        assert a in s, (p,a)
        s=s.replace(a,b,1)
    open(p,'w').write(s)

patch('support/minimig/minimig_a4091.cpp',[
('#include "minimig_a4091.h"\n','#include "minimig_a4091.h"\n#include "minimig_a2091.h"\n'),
("	int enabled = (minimig_config.scsi_cfg & 1);","	int enabled = (minimig_scsi_controller() == SCSI_CTRL_A4091);"),
])
patch('support/minimig/minimig_config.cpp',[
("""	printf("\\nA4091 SCSI: %s.\\n", (minimig_config.scsi_cfg & 1) ? "enabled" : "disabled");""",
 """	{
		static const char *ctrl_name[] = { "disabled", "A2091", "A4091" };
		printf("\\nSCSI controller: %s.\\n", ctrl_name[minimig_scsi_controller()]);
	}"""),
])
patch('menu.cpp',[
("""			// A4091 SCSI - Zorro III card, needs a 68020+. Inlined here (IDs
			// 1..6 scroll with the IDE drives, no sub-page). menusub 14, then
			// 15/16..25/26 for the six ID type/path pairs.
			if (minimig_config.cpu & 2)
			{
				MenuWrite(m++);   // gap between the IDE drives and A4091 SCSI
				int scsi_on = (minimig_config.scsi_cfg & 1) ? 1 : 0;
				strcpy(s, " A4091 SCSI       : ");
				strcat(s, scsi_on ? "Enabled" : "Disabled");""",
"""			// SCSI controller: A2091 (Zorro II, any CPU) or A4091 (Zorro III,
			// 68020+ only). Inlined here (IDs 1..6 scroll with the IDE drives,
			// no sub-page). menusub 14, then 15/16..25/26 for the six ID
			// type/path pairs. Takes effect on the next reset.
			int scsi_ctrl = minimig_scsi_controller();
			if (a2091_core_supported() || (minimig_config.cpu & 2))
			{
				MenuWrite(m++);   // gap between the IDE drives and SCSI
				int scsi_on = scsi_ctrl != SCSI_CTRL_NONE;
				strcpy(s, " SCSI controller  : ");
				strcat(s, scsi_ctrl == SCSI_CTRL_A2091 ? "A2091" : scsi_ctrl == SCSI_CTRL_A4091 ? "A4091" : "Disabled");"""),
("""			else if (menusub == 14) // A4091 SCSI enable/disable
			{
				if (select || minus || plus)
				{
					minimig_config.scsi_cfg ^= 1;
					a4091_apply_config();
					menustate = MENU_MINIMIG_DISK1;
				}
			}""",
"""			else if (menusub == 14) // SCSI controller: Disabled -> A2091 -> A4091 (68020+)
			{
				if (select || minus || plus)
				{
					// build the list of what this core / CPU can offer
					int opts[3], n = 0, cur = 0;
					opts[n++] = SCSI_CTRL_NONE;
					if (a2091_core_supported()) opts[n++] = SCSI_CTRL_A2091;
					if ((minimig_config.cpu & 2) && a4091_core_supported()) opts[n++] = SCSI_CTRL_A4091;
					int ctrl = minimig_scsi_controller();
					for (int k = 0; k < n; k++) if (opts[k] == ctrl) cur = k;
					cur = minus ? (cur + n - 1) % n : (cur + 1) % n;
					switch (opts[cur])
					{
					case SCSI_CTRL_NONE:  minimig_config.scsi_cfg &= ~1; break;
					case SCSI_CTRL_A2091: minimig_config.scsi_cfg |= 3; break;
					case SCSI_CTRL_A4091: minimig_config.scsi_cfg = (minimig_config.scsi_cfg | 1) & ~2; break;
					}
					a4091_apply_config();
					a2091_apply_config();
					menustate = MENU_MINIMIG_DISK1;
				}
			}"""),
])
s=open('menu.cpp').read()
if '#include "support/minimig/minimig_a2091.h"' not in s:
    s=s.replace('#include "support/minimig/minimig_a4091.h"\n','#include "support/minimig/minimig_a4091.h"\n#include "support/minimig/minimig_a2091.h"\n',1)
    assert 'minimig_a2091.h' in s
open('menu.cpp','w').write(s)
print("ok")
